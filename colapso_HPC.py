# -*- coding: utf-8 -*-

from abaqus import *
from abaqusConstants import *

from part import *
from material import *
from section import *
from assembly import *  
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
import regionToolset

from odbAccess import openOdb
import numpy as np
import random
import time
import os

import sys


# --------------------------------------------------------------------------
# DEFINIÇÕES DE FUNÇÕES
# --------------------------------------------------------------------------
def log(message):

    texto = str(message)

    # Imprime na área de mensagens do Abaqus/CAE
    print(texto)

    try:
        sys.stdout.flush()
    except Exception:
        pass

    # Imprime também no terminal que iniciou o Abaqus
    try:
        if sys.__stdout__ is not None and sys.__stdout__ is not sys.stdout:
            sys.__stdout__.write(texto + '\n')
            sys.__stdout__.flush()
    except Exception:
        pass

def get_tensao_max(id):

    odb_path = 'Job_Portico2D_HPC_'+repr(id)+'.odb'

    #Abrir Odb
    odb = openOdb(path=odb_path)

    #Pegar dados do ultimo step e frame
    lastStep = odb.steps.values()[-1]
    lastFrame = lastStep.frames[-1]

    #Salvar Tensões
    stress = lastFrame.fieldOutputs['S']

    von_mises = stress.getScalarField(invariant=MISES)

    values = von_mises.values
    stress_data = np.array([v.data for v in values])

    max_stress = np.max(stress_data)


    log("Tensao Maxima: " +repr(max_stress))

    odb.close()

    return max_stress

def get_desloc_max_x(id):

    odb_path = 'Job_Portico2D_HPC_'+repr(id)+'.odb'

    #Abrir Odb
    odb = openOdb(path=odb_path)

    #Pegar dados do ultimo step e frame
    lastStep = odb.steps.values()[-1]
    lastFrame = lastStep.frames[-1]

    #Salvar Deslocamento Máximo
    displacement = lastFrame.fieldOutputs['U']

    max_desloc = -1

    for value in displacement.values:
        u1, u2 = value.data[0], value.data[1]

        # magnitude = np.sqrt(u1**2 + u2**2)
        
        # if magnitude > max_desloc:
        #     max_desloc = magnitude

        if abs(u1) > max_desloc:
            max_desloc = abs(u1)

    odb.close()

    return max_desloc

def run_job(id):
     
    #Job
    job_name = 'Job_Portico2D_HPC_'+repr(id)
    model_name = 'Portico_2D_'+repr(id)

    if job_name in mdb.jobs.keys():
        del mdb.jobs[job_name]

    # Remove arquivos antigos para impedir a leitura de resultados de outro job
    arquivos_antigos = [
        job_name + '.odb',
        job_name + '.lck',
        job_name + '.sta',
        job_name + '.msg',
        job_name + '.dat'
    ]

    for arquivo in arquivos_antigos:
        if os.path.exists(arquivo):
            try:
                os.remove(arquivo)
            except Exception:
                pass

    job = mdb.Job(name=job_name, model=model_name, type=ANALYSIS, memory=90, memoryUnits=PERCENTAGE, numCpus=6, numDomains=6, multiprocessingMode=DEFAULT)    
    job.submit(consistencyChecking=OFF)
    job.waitForCompletion()

    if job.status != COMPLETED:
        raise RuntimeError("Job %s terminou com status %s" % (job_name, str(job.status)))

def gera_estrutura(sol, id):

    model_name = 'Portico_2D_' + repr(id)

    if model_name in mdb.models.keys():
        del mdb.models[model_name]

    mdb.Model(modelType=STANDARD_EXPLICIT, name=model_name )

    model = mdb.models[model_name]

    # Parametros Geometricos (SI: metros)

    # Geometria 
    s = model.ConstrainedSketch(name='__profile__', sheetSize=50.0)

    # Desenhando as vigas
    idx_viga = num_pilares
    for j in range(1, pavimentos+1):
        y = j * H
        for i in range(blocos):
            x_start = i * L
            x_end = (i + 1) * L
            
            if sol[idx_viga] != -1:
                s.Line(point1=(x_start, y), point2=(x_end, y))
                
            idx_viga += 1

    pilares = []
    # Desenhando os pilares
    idx_pilar = 0
    for i in range(blocos+1):
        x = i * L
        for j in range(pavimentos):
            y_start = j * H
            y_end = (j + 1) * H
            
            if sol[idx_pilar] != -1:
                s.Line(point1=(x, y_start), point2=(x, y_end))
                pilares.append((x, y_start, y_end))
                
            idx_pilar += 1

    # Criando a Part
    part = model.Part(name='Estrutura', dimensionality=TWO_D_PLANAR, type=DEFORMABLE_BODY)
    part.BaseWire(sketch=s)

    for i, (x, y_start, y_end) in enumerate(pilares):
        part.Set(edges=part.edges.findAt(((x, (y_start + y_end) / 2.0, 0.0),)), name='Pilar_%d' % i)

    del model.sketches['__profile__']

    # SETS
    beam_loc = []
    col_loc = []

    idx_pilar = 0
    for i in range(blocos + 1):
        for j in range(pavimentos):
            if sol[idx_pilar] != -1: 
                coordenada = ((i * L, j * H + (H / 2.0), 0.0),)
                col_loc.append(coordenada)
            idx_pilar += 1

    idx_viga = num_pilares
    for j in range(1, pavimentos+1):
        for i in range(blocos):
            if sol[idx_viga] != -1:
                coordenada = ((i * L + (L / 2.0), j * H, 0.0),)
                beam_loc.append(coordenada)
            idx_viga += 1

    col_edges = part.edges.findAt(*col_loc)
    beam_edges = part.edges.findAt(*beam_loc)

    set_vigas = part.Set(edges=beam_edges, name='Grupo_Vigas')
    set_pilares = part.Set(edges=col_edges, name='Grupo_Pilares')

    # DEFINIÇÃO DOS MATERIAIS
    # Material Base: Concreto Convencional
    model.Material(name='Concreto_Original')
    model.materials['Concreto_Original'].Elastic(table=((30E9, 0.2),))
    
    # Material de Reforço: Concreto de Alto Desempenho (HPC)
    model.Material(name='Concreto_HPC')
    model.materials['Concreto_HPC'].Elastic(table=((50E9, 0.2),))

    model.RectangularProfile(name='Perfil_Padrao', a=h_padrao, b=b_padrao)

    # --------------------------------------------------------------------------
    # Atribuição Dinâmica para os Pilares
    # --------------------------------------------------------------------------
    idx_pilar = 0
    idx_geometria_existente = 0

    for i in range(blocos + 1):
        for j in range(pavimentos):
            
            bit_ativo = sol[idx_pilar]
            
            if bit_ativo != -1:
                pilar_edge = part.edges.findAt(col_loc[idx_geometria_existente])
                pilar_region = regionToolset.Region(edges=pilar_edge)
                
                if bit_ativo == 1:
                    # Mapeamento do vetor 3N
                    h_val = sol[idx_pilar + N]
                    b_val = sol[idx_pilar + 2*N]
                    
                    # Nome único para o perfil baseado em suas dimensões
                    perfil_nome = 'Perf_R_%.3fx%.3f' % (h_val, b_val)
                    secao_pilar_name = 'Secao_' + perfil_nome
                    
                    if perfil_nome not in model.profiles.keys():
                        model.RectangularProfile(name=perfil_nome, a=h_val, b=b_val)
                    
                    if secao_pilar_name not in model.sections.keys():
                        model.BeamSection(
                            name=secao_pilar_name,
                            integration=DURING_ANALYSIS,
                            poissonRatio=0.2,
                            profile=perfil_nome,
                            material='Concreto_HPC', # Mudança automática para HPC se reforçado
                            temperatureVar=LINEAR
                        )
                    
                    part.SectionAssignment(region=pilar_region, sectionName=secao_pilar_name)
                    
                elif bit_ativo == 0:
                    if 'Secao_Padrao' not in model.sections.keys():
                        model.BeamSection(
                            name='Secao_Padrao',
                            integration=DURING_ANALYSIS,
                            poissonRatio=0.2,
                            profile='Perfil_Padrao',
                            material='Concreto_Original',
                            temperatureVar=LINEAR
                        )
                    
                    part.SectionAssignment(region=pilar_region, sectionName='Secao_Padrao')
                
                idx_geometria_existente += 1
                
            idx_pilar += 1

    # --------------------------------------------------------------------------
    # Atribuição Dinâmica para as Vigas
    # --------------------------------------------------------------------------
    idx_viga = num_pilares
    idx_geometria_viga_existente = 0

    for j in range(1, pavimentos+1):
        for i in range(blocos):
            
            bit_ativo = sol[idx_viga]
            
            if bit_ativo != -1:
                viga_edge = part.edges.findAt(beam_loc[idx_geometria_viga_existente])
                viga_region = regionToolset.Region(edges=viga_edge)
                
                if bit_ativo == 1:
                    h_val = sol[idx_viga + N]
                    b_val = sol[idx_viga + 2*N]
                    
                    perfil_nome = 'Perf_R_%.3fx%.3f' % (h_val, b_val)
                    secao_viga_name = 'Secao_' + perfil_nome
                    
                    if perfil_nome not in model.profiles.keys():
                        model.RectangularProfile(name=perfil_nome, a=h_val, b=b_val)
                    
                    if secao_viga_name not in model.sections.keys():
                        model.BeamSection(
                            name=secao_viga_name,
                            integration=DURING_ANALYSIS,
                            poissonRatio=0.2,
                            profile=perfil_nome,
                            material='Concreto_HPC', # Mudança automática para HPC se reforçado
                            temperatureVar=LINEAR
                        )
                    
                    part.SectionAssignment(region=viga_region, sectionName=secao_viga_name)
                    
                elif bit_ativo == 0:
                    if 'Secao_Padrao' not in model.sections.keys():
                        model.BeamSection(
                            name='Secao_Padrao',
                            integration=DURING_ANALYSIS,
                            poissonRatio=0.2,
                            profile='Perfil_Padrao',
                            material='Concreto_Original',
                            temperatureVar=LINEAR
                        )
                    
                    part.SectionAssignment(region=viga_region, sectionName='Secao_Padrao')
                
                idx_geometria_viga_existente += 1
                
            idx_viga += 1

    # Orientacao das secoes
    part.assignBeamSectionOrientation(region=set_pilares, method=N1_COSINES, n1=(0.0, 0.0, -1.0))
    part.assignBeamSectionOrientation(region=set_vigas, method=N1_COSINES, n1=(0.0, 0.0, -1.0))

    # Assembly e Step
    asm = model.rootAssembly
    inst = asm.Instance(name='Estrutura_Inst', part=part, dependent=ON)

    model.StaticStep(name='Step_Cargas', previous='Initial', description='Aplicacao de cargas')

    # Condições de contorno
    base_loc = []
    for i in range(blocos+1):
        coordenada = ((i*L, 0.0, 0.0),)   
        base_loc.append(coordenada) 

    base_verts = inst.vertices.findAt(*base_loc)
    region_base = regionToolset.Region(vertices=base_verts)

    model.DisplacementBC(name='Apoios_Base', createStepName='Initial', region=region_base, u1=0.0, u2=0.0, ur3=UNSET)

    # Cargas
    beam_edges_inst = inst.edges.findAt(*beam_loc)
    region_vigas_load = regionToolset.Region(edges=beam_edges_inst)
    model.LineLoad(name='Carga_Gravitacional', createStepName='Step_Cargas', region=region_vigas_load, comp2=-40860.89)

    # Cargas laterais: 5 kip, 5 kip e 2.5 kip
    no_andar1 = inst.vertices.findAt(
        ((0.0, H, 0.0),)
    )

    model.ConcentratedForce(
        name='Carga_Lat_Andar1',
        createStepName='Step_Cargas',
        region=regionToolset.Region(vertices=no_andar1),
        cf1=22240.0
    )

    no_andar2 = inst.vertices.findAt(
        ((0.0, 2.0 * H, 0.0),)
    )

    model.ConcentratedForce(
        name='Carga_Lat_Andar2',
        createStepName='Step_Cargas',
        region=regionToolset.Region(vertices=no_andar2),
        cf1=22240.0
    )

    no_topo = inst.vertices.findAt(
        ((0.0, 3.0 * H, 0.0),)
    )

    model.ConcentratedForce(
        name='Carga_Lat_Topo',
        createStepName='Step_Cargas',
        region=regionToolset.Region(vertices=no_topo),
        cf1=11120.0
    )

    # Malha
    all_edges = part.edges[:]
    part.seedEdgeByNumber(edges=all_edges, number=10, constraint=FIXED)
    elemTypeBeam = ElemType(elemCode=B21, elemLibrary=STANDARD)
    part.setElementType(regions=(all_edges,), elemTypes=(elemTypeBeam,))
    part.generateMesh()
    asm.regenerate()

    return model

def salva_info(path, sol, max_desloc_x, g, id, pilar_rem):

    f = open(path, 'a')

    f.write(repr(g)+'; ')   
    
    f.write(repr(id)+'; ')   

    f.write(repr(pilar_rem)+'; ')

    for i in range(len(sol)):
        f.write(repr(sol[i])+'; ')
    
    f.write(repr(max_desloc_x)+'; ')   
    
    f.write(repr(get_custo(sol))+';')

    if max_desloc_x < limite_desloc:
        f.write(" VIAVEL")
    else:
        f.write(" INVIAVEL")

    f.write('\n')
    f.close()
    
def salva_populacao(path, populacao):
    
    f = open(path, 'w')
    
    for i in range(len(populacao)):    
        for j in range(3*N):
            if j < 3*N-1:
                f.write(repr(populacao[i][j])+"; ")
            else:
                f.write(repr(populacao[i][j]))
        f.write("\n")
    f.close()

def limpa_execucao(id):

    job_name = 'Job_Portico2D_HPC_' + repr(id)
    model_name = 'Portico_2D_' + repr(id)

    if job_name in mdb.jobs.keys():
        del mdb.jobs[job_name]

    if model_name in mdb.models.keys():
        del mdb.models[model_name]

    arquivos = [
        job_name + '.odb',
        job_name + '.lck',
        job_name + '.sta',
        job_name + '.msg',
        job_name + '.dat',
        job_name + '.com',
        job_name + '.prt',
        job_name + '.sim'
    ]

    for arquivo in arquivos:
        if os.path.exists(arquivo):
            try:
                os.remove(arquivo)
            except Exception:
                pass    
    
def salva_melhores(path, geracao, melhor, fitness):
    
    f = open(path, 'a')
    
    f.write(repr(geracao) + "; ")
      
    for j in range(3*N):
            f.write(repr(melhor[j])+"; ")        
                
    f.write(repr(fitness))

    f.write("\n")
    f.close()

def get_custo(sol):

    volume_HPC = 0.0
    volume_concreto = 0.0

    for i in range(0,N):

        if sol[i] == -1:
            pass
            
        elif i < num_pilares:
            if sol[i] == 1:
                volume_HPC += (sol[i+N] * sol[i+2*N]) * H

            else:
                volume_concreto += (b_padrao * h_padrao) * H

        else: 
            if sol[i] == 1:
                volume_HPC +=  (sol[i+N] * sol[i+2*N]) * L
            else:
                volume_concreto += (b_padrao * h_padrao) * L

    # volume_total = volume_HPC + volume_concreto

    custo_total = volume_HPC * CUSTO_HPC_M3 + volume_concreto * CUSTO_CONCRETO_M3
            
    # log("Custo:"+repr(custo_total))
    
    return custo_total

def gera_individuo():
    individuo = [0] * (3*N)
    
    for i in range(N):
        individuo[i] = random.randint(0, 1) # Define reforço (0 ou 1)
        
        # Sorteando floats para h e b uniformemente dentro do range
        individuo[i + N] = round(random.uniform(h_min, h_max),2)
        individuo[i + 2*N] = round(random.uniform(b_min, b_max),2)
    
    return individuo

def gera_individuo_reforcado():
    individuo = [1] * (3*N)
    
    for i in range(N):
        # Define o primeiro indivíduo com o reforço máximo permitido
        # individuo[i + N] = round(random.uniform(h_max-(0.1*h_max), h_max),2)
        # individuo[i + 2*N] = round(random.uniform(b_max-(0.1*b_max), b_max),2)
        individuo[i + N] = h_max
        individuo[i + 2*N] = b_max
    
    return individuo

def remove_pilares(model, path, sol, g, id_individuo, run_id):
    penalidade_total = 0.0
    pior_desloc_remocao = None
    pior_caso_remocao = None
    asm = model.rootAssembly
    inst = asm.instances['Estrutura_Inst']

    for i in range(num_pilares):
        set_name = 'Pilar_%d' % i
        log("Removendo pilar %d" % i)

        if set_name not in inst.sets.keys():
            continue  # Pular se o pilar não existir na instância

        inter = model.ModelChange(name="REM", createStepName='Step_Cargas', region=inst.sets[set_name], activeInStep=False, includeStrain=False)

        try:
            run_job(run_id)
            deslocamento_max = get_desloc_max_x(run_id)

            salva_info(path, sol, deslocamento_max, g, id_individuo, i)

            if pior_desloc_remocao is None or deslocamento_max > pior_desloc_remocao:
                pior_desloc_remocao = deslocamento_max
                pior_caso_remocao = 'remocao_pilar_%d' % i

            if deslocamento_max > limite_desloc:
                penalidade_total += 1e6 * (deslocamento_max - limite_desloc)

        except Exception as e:
            log("Erro ao remover pilar %d: %s" % (i, repr(e)))
            penalidade_total += 1e8  # Penalidade máxima se ocorrer erro

        finally:
            if 'REM' in model.interactions.keys():
                del model.interactions['REM']

    return penalidade_total, pior_desloc_remocao, pior_caso_remocao

def avalia_individuo(g, sol, id):
    run_id = None

    try:
        tempo_ind_inicio = time.time()
        penalidade_total = 0.0

        run_id = g * 1000 + id

        model = gera_estrutura(sol, run_id)
        run_job(run_id)

        max_desloc_x = get_desloc_max_x(run_id)

        pior_desloc = max_desloc_x
        pior_caso = "estrutura_completa"

        if max_desloc_x > limite_desloc:
            penalidade_total += 1e6 * (max_desloc_x - limite_desloc)

        salva_info(path, sol, max_desloc_x, g, id, -1)

        penalidade_rem, pior_desloc_rem, pior_caso_rem = remove_pilares(model, path, sol, g, id, run_id)

        penalidade_total += penalidade_rem

        if pior_desloc_rem is not None and pior_desloc_rem > pior_desloc:
            pior_desloc = pior_desloc_rem
            pior_caso = pior_caso_rem

        custo = get_custo(sol)

        if penalidade_total == 0.0:
            viabilidade = "VIAVEL"
            fitness = custo
        else:
            viabilidade = "INVIAVEL"
            fitness = PENALIDADE_BASE + custo + penalidade_total

        tempo_ind = time.time() - tempo_ind_inicio

        log("\nIndividuo: " + repr(id))
        log("Pior caso: " + repr(pior_caso))
        log("Pior deslocamento maximo X: " + repr(pior_desloc))
        log("Custo: " + repr(custo))
        log("Viabilidade: " + viabilidade)
        log("Penalidade total: " + repr(penalidade_total))
        log("Fitness: " + repr(fitness))
        log("Tempo individuo: %.2f s" % tempo_ind)

        return fitness, viabilidade, pior_desloc, custo

    except Exception as e:
        log("Erro ao avaliar individuo " + repr(id) + ": " + repr(e))
        return float('inf'), "ERRO", None, None

    finally:
        if run_id is not None:
            try:
                limpa_execucao(run_id)
            except Exception as e:
                log("Aviso: erro durante a limpeza: " + repr(e))

def seleciona_pai_torneio(populacao, fitnesses, k=3):

    quantidade = min(k, len(populacao))
    participantes = random.sample(range(len(populacao)), quantidade)
    melhor_indice = min(participantes, key=lambda i: fitnesses[i])

    return populacao[melhor_indice][:]

def cross_over(pai1, pai2):
    ponto_corte = random.randint(1, len(pai1) - 2)
    filho1 = pai1[:ponto_corte] + pai2[ponto_corte:]
    filho2 = pai2[:ponto_corte] + pai1[ponto_corte:]
    return filho1, filho2

def mutacao(individuo, taxa_mutacao=0.01):

    for i in range(3*N):
        valor_atual = individuo[i]
        if random.random() < taxa_mutacao:
            if i < N:  # Mutação para bits binários
                individuo[i] = 1 - valor_atual  # Inverte o bit
            elif N <= i < 2*N:  # Mutação para h
                individuo[i] = round(random.uniform(max(valor_atual * 0.85, h_min), min(valor_atual * 1.15, h_max)), 2)
            else:  # Mutação para b
                individuo[i] = round(random.uniform(max(valor_atual * 0.85, b_min), min(valor_atual * 1.15, b_max)), 2)
    return individuo



# --------------------------------------------------------------------------
# CONFIGURAÇÃO GERAL 
# --------------------------------------------------------------------------
pavimentos = 3
blocos = 2

L = 10.9728  
H = 3.048  

num_pilares = (blocos+1)*pavimentos
num_vigas = blocos*pavimentos
N = num_pilares + num_vigas

path = "dados.csv"
path_pop = "last_populacao.csv"
path_bests = "bests.csv"

f = open(path, 'w')

cabecalho = [
    'geracao',
    'individuo',
    'pilar_removido'
]

for i in range(N):
    cabecalho.append('reforco_%d' % i)

for i in range(N):
    cabecalho.append('h_%d' % i)

for i in range(N):
    cabecalho.append('b_%d' % i)

cabecalho.extend([
    'deslocamento_max',
    'custo',
    'status_cenario'
])

f.write('; '.join(cabecalho) + '\n')
f.close()
f = open(path_bests, 'w')
f.close()

#Desativar quando não for mais teste!!!
random.seed(42)

# Tamanho 3N: [0..N-1] -> Binário, [N..2N-1] -> h, [2N..3N-1] -> b
sol = [0] * (3*N)   


# Perfil Retangular Padrão Geométrico Base
h_padrao = 0.40
b_padrao = 0.30

# Intervalos de variação para o Concreto
h_min, h_max = 0.01, 1.0
b_min, b_max = 0.01, 1.0
penalidade = 0.0

limite_desloc = 0.03048

#Custo Materiais (Custo por metro cubico)
CUSTO_HPC_M3 = 13000.0
CUSTO_CONCRETO_M3 = 500.0

# Penalidade maior que o custo máximo teórico da estrutura
comprimento_total = num_pilares * H + num_vigas * L
custo_maximo_teorico = comprimento_total * h_max * b_max * CUSTO_HPC_M3
PENALIDADE_BASE = 10.0 * custo_maximo_teorico

#Configurações GA
tamanho_pop = 20
num_geracoes = 10
taxa_crossover = 0.8
numero_elites = 2


# --------------------------------------------------------------------------
# EXECUÇÃO
# --------------------------------------------------------------------------
tempo_inicio = time.time()
populacao = []
melhor_global = None
melhor_fitness = float('inf')
melhor_desloc = None
melhor_custo = None
melhor_viabilidade = None

melhor_viavel_global = None
melhor_custo_viavel = float('inf')
melhor_desloc_viavel = None
melhor_fitness_viavel = float('inf')

for i in range(tamanho_pop):
    if i == 0:
        individuo = gera_individuo_reforcado()
    else:
        individuo = gera_individuo()
    populacao.append(individuo)
 

for geracao in range(num_geracoes):
    fitnesses = []
    
    salva_populacao(path_pop, populacao)

    log("\n\nGERACAO " + repr(geracao) + ':')

    for idx, individuo in enumerate(populacao):
        run_id = geracao * tamanho_pop + idx
        fitness, viabilidade, max_desloc_x, custo = avalia_individuo(geracao, individuo, run_id)
        fitnesses.append(fitness)

        # Melhor indivíduo geral
        if fitness < melhor_fitness:
            melhor_fitness = fitness
            melhor_global = individuo[:]
            melhor_desloc = max_desloc_x
            melhor_custo = custo
            melhor_viabilidade = viabilidade

        # Melhor indivíduo realmente viável
        if viabilidade == "VIAVEL" and custo is not None and custo < melhor_custo_viavel:
            melhor_viavel_global = individuo[:]
            melhor_custo_viavel = custo
            melhor_desloc_viavel = max_desloc_x
            melhor_fitness_viavel = fitness

    # Elitismo: copia os melhores sem crossover e sem mutação
    indices_ordenados = sorted(range(len(populacao)), key=lambda i: fitnesses[i])
    indices_elite = indices_ordenados[:numero_elites]

    nova_populacao = []

    for indice in indices_elite:
        nova_populacao.append(populacao[indice][:])

    while len(nova_populacao) < tamanho_pop:
        pai1 = seleciona_pai_torneio(populacao, fitnesses, k=3)
        pai2 = seleciona_pai_torneio(populacao, fitnesses, k=3)

        if random.random() < taxa_crossover:
            filho1, filho2 = cross_over(pai1, pai2)
        else:
            filho1, filho2 = pai1[:], pai2[:]

        filho1 = mutacao(filho1)
        filho2 = mutacao(filho2)

        nova_populacao.append(filho1)

        if len(nova_populacao) < tamanho_pop:
            nova_populacao.append(filho2)

    populacao = nova_populacao
    
    if melhor_viavel_global is not None:
        salva_melhores(path_bests, geracao, melhor_viavel_global, melhor_fitness_viavel)
    elif melhor_global is not None:
        salva_melhores(path_bests, geracao, melhor_global, melhor_fitness)


tempo_total = time.time() - tempo_inicio

if melhor_viavel_global is not None:
    log("\n\nMELHOR SOLUCAO VIAVEL ENCONTRADA")
    log("Melhor fitness viavel: " + repr(melhor_fitness_viavel))
    log("Melhor custo viavel: " + repr(melhor_custo_viavel))
    log("Pior deslocamento da solucao viavel: " + repr(melhor_desloc_viavel))
    log("Viabilidade final: VIAVEL")
    log("Melhor individuo viavel:")
    log(melhor_viavel_global)
else:
    log("\n\nNENHUMA SOLUCAO VIAVEL FOI ENCONTRADA")
    log("Melhor fitness geral: " + repr(melhor_fitness))
    log("Melhor custo geral: " + repr(melhor_custo))
    log("Pior deslocamento geral: " + repr(melhor_desloc))
    log("Viabilidade: " + repr(melhor_viabilidade))
    log("Melhor individuo encontrado:")
    log(melhor_global)

horas = int(tempo_total // 3600)
minutos = int((tempo_total % 3600) // 60)
segundos = tempo_total % 60

log("Tempo total de execucao: %d h %d min %.2f s" % (horas, minutos, segundos))