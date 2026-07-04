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


def gera_estrutura(sol, id):

    mdb.Model(modelType=STANDARD_EXPLICIT, name='Portico_2D_'+repr(id))
    model = mdb.models['Portico_2D_'+repr(id)]

    # Parametros Geometricos (SI: metros)
    L = 10.9728  
    H = 3.048    

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

    # Desenhando os pilares
    idx_pilar = 0
    for i in range(blocos+1):
        x = i * L
        for j in range(pavimentos):
            y_start = j * H
            y_end = (j + 1) * H
            
            if sol[idx_pilar] != -1:
                s.Line(point1=(x, y_start), point2=(x, y_end))
                
            idx_pilar += 1

    # Criando a Part
    part = model.Part(name='Estrutura', dimensionality=TWO_D_PLANAR, type=DEFORMABLE_BODY)
    part.BaseWire(sketch=s)
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

    # Perfil Retangular Padrão Geométrico Base
    h_padrao = 0.40
    b_padrao = 0.30
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
                    perfil_nome = 'Perf_Pilar_R_%.3fx%.3f' % (h_val, b_val)
                    secao_pilar_name = 'Secao_Pilar_' + perfil_nome
                    
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
                    if 'Secao_Pilar_Padrao' not in model.sections.keys():
                        model.BeamSection(
                            name='Secao_Pilar_Padrao',
                            integration=DURING_ANALYSIS,
                            poissonRatio=0.2,
                            profile='Perfil_Padrao',
                            material='Concreto_Original',
                            temperatureVar=LINEAR
                        )
                    
                    part.SectionAssignment(region=pilar_region, sectionName='Secao_Pilar_Padrao')
                
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
                    
                    perfil_nome = 'Perf_Viga_R_%.3fx%.3f' % (h_val, b_val)
                    secao_viga_name = 'Secao_Viga_' + perfil_nome
                    
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
                    if 'Secao_Viga_Padrao' not in model.sections.keys():
                        model.BeamSection(
                            name='Secao_Viga_Padrao',
                            integration=DURING_ANALYSIS,
                            poissonRatio=0.2,
                            profile='Perfil_Padrao',
                            material='Concreto_Original',
                            temperatureVar=LINEAR
                        )
                    
                    part.SectionAssignment(region=viga_region, sectionName='Secao_Viga_Padrao')
                
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

    # Malha
    all_edges = part.edges[:]
    part.seedEdgeByNumber(edges=all_edges, number=10, constraint=FIXED)
    elemTypeBeam = ElemType(elemCode=B21, elemLibrary=STANDARD)
    part.setElementType(regions=(all_edges,), elemTypes=(elemTypeBeam,))
    part.generateMesh()
    asm.regenerate()

    # Job
    job_name = 'Job_Portico2D_HPC_'+repr(id)
    if job_name in mdb.jobs.keys():
        del mdb.jobs[job_name]

    job = mdb.Job(name=job_name, model='Portico_2D_'+repr(id), type=ANALYSIS, memory=90, memoryUnits=PERCENTAGE)
    job.submit(consistencyChecking=OFF)
    job.waitForCompletion()
    
    # Visualização e plot do resultado em imagem

    try: 

        o3 = session.openOdb(name='C:/Users/victo/Desktop/Metodos/temp/Job_Portico2D_HPC_'+repr(id)+'.odb')
        session.viewports['Viewport: 1'].setValues(displayedObject=o3)
        a = mdb.models['Portico_2D_'+repr(id)].rootAssembly
        session.viewports['Viewport: 1'].setValues(displayedObject=a)
        session.viewports['Viewport: 1'].assemblyDisplay.setValues(optimizationTasks=OFF, geometricRestrictions=OFF, stopConditions=OFF)
        o7 = session.odbs['C:/Users/victo/Desktop/Metodos/temp/Job_Portico2D_HPC_'+repr(id)+'.odb']
        session.viewports['Viewport: 1'].setValues(displayedObject=o7)
        session.viewports['Viewport: 1'].odbDisplay.basicOptions.setValues(renderBeamProfiles=ON)
        session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(CONTOURS_ON_DEF, ))
        session.viewports['Viewport: 1'].viewportAnnotationOptions.setValues(triad=OFF, title=OFF, state=OFF, annotations=OFF, compass=OFF, legend=ON)
        session.viewports['Viewport: 1'].view.fitView()
        session.viewports['Viewport: 1'].view.setValues(nearPlane=44.7674, 
            farPlane=53.2767, width=29.6223, height=13.7368, viewOffsetX=-0.10578, 
            viewOffsetY=0.27568)
        session.viewports['Viewport: 1'].view.setValues(nearPlane=44.5265, 
            farPlane=53.5176, width=29.4629, height=13.6629, viewOffsetX=-1.63575, 
            viewOffsetY=0.248754)
        session.printToFile(fileName='C:/Users/victo/Desktop/Metodos/imagens/Job_Portico2D_HPC_'+repr(id)+'.png', format=PNG, canvasObjects=(session.viewports['Viewport: 1'], ))

    except Exception as e:
                                print('Erro ao salvar Viewport') 


# --------------------------------------------------------------------------
# CONFIGURAÇÃO GERAL E EXECUÇÃO DO LOOP PRINCIPAL
# --------------------------------------------------------------------------
pavimentos = 3
blocos = 2

import random

#Desativar quando não for mais teste!!!
random.seed(42)
    
num_pilares = (blocos+1)*pavimentos
num_vigas = blocos*pavimentos
N = num_pilares + num_vigas

# Tamanho 3N: [0..N-1] -> Binário, [N..2N-1] -> h, [2N..3N-1] -> b
sol = [0] * (3*N)   

# Intervalos de variação para o Concreto (Ex: entre 30cm e 80cm)
h_min, h_max = 0.30, 0.80
b_min, b_max = 0.20, 0.50

# Preenchendo o vetor solução de forma contínua
for i in range(N):
    sol[i] = random.randint(0, 1) # Define reforço (0 ou 1)
    
    # Sorteando floats para h e b uniformemente dentro do range
    sol[i + N] = random.uniform(h_min, h_max)
    sol[i + 2*N] = random.uniform(b_min, b_max)

# Execução do loop de simulações com remoção progressiva
for i in range(num_pilares + 1):

    #Gera e estrutura completa
    if i == num_pilares:
        gera_estrutura(sol, -1)
        break

    sol_local = sol[:]
    sol_local[i] = -1
    print(sol_local)
    gera_estrutura(sol_local, i)