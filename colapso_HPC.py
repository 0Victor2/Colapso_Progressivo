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

import sys


# --------------------------------------------------------------------------
# DEFINIÇÕES DE FUNÇÕES
# --------------------------------------------------------------------------
def log(message):
    print(str(message) + '\n')

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
    if job_name in mdb.jobs.keys():
        del mdb.jobs[job_name]

    job = mdb.Job(name=job_name, model='Portico_2D_'+repr(id), type=ANALYSIS, memory=90, memoryUnits=PERCENTAGE, numCpus=6, numDomains=6, multiprocessingMode=DEFAULT)    
    job.submit(consistencyChecking=OFF)
    job.waitForCompletion()

def gera_estrutura(sol, id):

    mdb.Model(modelType=STANDARD_EXPLICIT, name='Portico_2D_'+repr(id))
    model = mdb.models['Portico_2D_'+repr(id)]

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
        f.write("VIAVEL")
    else:
        f.write("INVIAVEL")

    f.write('\n')
    f.close()

def get_custo(sol):

    volume_HPC = 0
    volume_concreto = 0

    #Custo Materiais (Custo por metro cubico)
    custo_HPC = 13000
    custo_concreto = 500

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

    custo_total = volume_HPC * custo_HPC + volume_concreto * custo_concreto
            
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

def gera_individuo_viavel():
    individuo = [1] * (3*N)
    
    for i in range(N):
        # Sorteando floats para h e b uniformemente dentro do range
        individuo[i + N] = round(random.uniform(h_max-(0.1*h_max), h_max),2)
        individuo[i + 2*N] = round(random.uniform(b_max-(0.1*b_max), b_max),2)
        # individuo[i + N] = h_max
        # individuo[i + 2*N] = b_max
    
    return individuo

def remove_pilares(model, path, sol, g, id):
    penalidade_total = 0.0
    pior_desloc_remocao = None
    pior_caso_remocao = None
    asm = model.rootAssembly
    inst = asm.instances['Estrutura_Inst']
    for i in range(num_pilares):
        set_name = 'Pilar_%d' % i
        print("Removendo pilar %d" % i)

        if set_name not in inst.sets.keys():
            continue  # Pular se o pilar não existir na instância

        inter = model.ModelChange(name="REM", createStepName='Step_Cargas', region=inst.sets[set_name], activeInStep=False, includeStrain=False)
        try:
            run_job(id)
            deslocamento_max = get_desloc_max_x(id)
            salva_info(path, sol, deslocamento_max, g, id, i)
            if pior_desloc_remocao is None or deslocamento_max > pior_desloc_remocao:
                pior_desloc_remocao = deslocamento_max
                pior_caso_remocao = 'remocao_pilar_%d' % i
            if deslocamento_max > limite_desloc:
                penalidade_total += 1e6 * (deslocamento_max - limite_desloc)
        except Exception as e:
            log("Erro ao remover pilar %d: %s" % (i, repr(e)))
            penalidade_total += 1e8  # Penalidade máxima se ocorrer erro
        del model.interactions['REM']

    return penalidade_total, pior_desloc_remocao, pior_caso_remocao

def avalia_individuo(g, sol, id):
    try:
        tempo_ind_inicio = time.time()

        # limite_desloc = limite_desloc
        penalidade_total = 0.0

        # Avalia a estrutura original primeiro
        run_id = g * 1000 + id  # Garante que cada execução tenha um ID único

        # model = gera_estrutura(sol, run_id)
        model = gera_estrutura(sol, run_id)
        run_job(run_id)

        max_desloc_x = get_desloc_max_x(run_id)

        pior_desloc = max_desloc_x
        pior_caso = "estrutura_completa"

        if max_desloc_x > limite_desloc:
            penalidade_total += 1e6 * (max_desloc_x - limite_desloc)

        salva_info(path, sol, max_desloc_x, g, id, -1)

        # Remocao progressiva dos pilares
        penalidade_rem, pior_desloc_rem, pior_caso_rem = remove_pilares(model, path, sol, g, run_id)

        penalidade_total += penalidade_rem

        if pior_desloc_rem is not None and pior_desloc_rem > pior_desloc:
            pior_desloc = pior_desloc_rem
            pior_caso = pior_caso_rem

        custo = get_custo(sol)

        if penalidade_total == 0.0:
            viabilidade = "VIAVEL"
        else:
            viabilidade = "INVIAVEL"

        fitness = custo + penalidade_total
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

def torneio(populacao, fitnesses, k=3):

    bests = populacao[:]
    bests_fitnesses = fitnesses[:]

    sorted_indices = sorted(range(len(bests_fitnesses)), key=lambda i: bests_fitnesses[i])

    return [bests[i] for i in sorted_indices[:(k-1)]]

def best_population(populacao_plus, fitnesses_plus, n_pop):
    sorted_indices = sorted(range(len(fitnesses_plus)), key=lambda i: fitnesses_plus[i])
    best_individuals = [populacao_plus[i] for i in sorted_indices[:n_pop]]
    best_fitnesses = [fitnesses_plus[i] for i in sorted_indices[:n_pop]]
    return best_individuals, best_fitnesses

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
                individuo[i] = round(random.uniform(max(valor_atual * 0.85, h_min), min(individuo[i+N]*1.15, h_max)), 2)
            else:  # Mutação para b
                individuo[i] = round(random.uniform(max(valor_atual * 0.85, b_min), min(individuo[i-N]*1.15, b_max)), 2)
    return individuo



# --------------------------------------------------------------------------
# CONFIGURAÇÃO GERAL 
# --------------------------------------------------------------------------
pavimentos = 3
blocos = 2

L = 10.9728  
H = 3.048  

path = "dados.csv"

f = open(path, 'w')
f.close()

#Desativar quando não for mais teste!!!
random.seed(42)
    
num_pilares = (blocos+1)*pavimentos
num_vigas = blocos*pavimentos
N = num_pilares + num_vigas

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

#Configurações GA
tamanho_pop = 20
num_geracoes = 10
taxa_crossover = 0.8


# --------------------------------------------------------------------------
# EXECUÇÃO
# --------------------------------------------------------------------------
tempo_inicio = time.time()
populacao = []
old_populacao = []
fitnesses_old = []
melhor_global, melhor_fitness, melhor_desloc, melhor_custo = None, float('inf'), None, None

for i in range(tamanho_pop):
    if i <= 3:
        individuo = gera_individuo_viavel()
    else:
        individuo = gera_individuo()
    populacao.append(individuo)

for geracao in range(num_geracoes):
    fitnesses = []

    log("\n\nGERACAO " + repr(geracao) + ':')

    for idx, individuo in enumerate(populacao):
        run_id = geracao * tamanho_pop + idx
        fitness, viabilidade, max_desloc_x, custo = avalia_individuo(geracao, individuo, run_id)
        fitnesses.append(fitness)

        if fitness < melhor_fitness:
            melhor_fitness = fitness
            melhor_global = individuo[:]
            melhor_desloc = max_desloc_x
            melhor_custo = custo

    nova_populacao = []
    populacao_plus = populacao + old_populacao
    fitnesses_plus = fitnesses + fitnesses_old

    while len(nova_populacao) < tamanho_pop:
        pais, pais_fitnesses = best_population(populacao_plus, fitnesses_plus, tamanho_pop)
        bests = torneio(pais, pais_fitnesses)
        pai1 = random.choice(bests)
        pai2 = random.choice(bests)
        while pai1 == pai2:
            pai2 = random.choice(bests)

        if random.random() < taxa_crossover:
            filho1, filho2 = cross_over(pai1, pai2)
        else:
            filho1, filho2 = pai1[:], pai2[:]

        filho1 = mutacao(filho1)
        filho2 = mutacao(filho2)

        nova_populacao.append(filho1)

        if len(nova_populacao) < tamanho_pop:
            nova_populacao.append(filho2)

    old_populacao = populacao
    fitnesses_old = fitnesses
    populacao = nova_populacao


tempo_total = time.time() - tempo_inicio

log("\n\nMELHOR SOLUCAO ENCONTRADA")
log("Melhor fitness: " + repr(melhor_fitness))
log("Melhor custo: " + repr(melhor_custo))
log("Melhor deslocamento maximo X: " + repr(melhor_desloc))

if melhor_desloc is not None and melhor_desloc < limite_desloc:
    log("Viabilidade final: VIAVEL")
else:
    log("Viabilidade final: INVIAVEL")

log("Melhor individuo:")
log(melhor_global)

log("Tempo total de execucao: %.2f min" % (tempo_total / 60.0))