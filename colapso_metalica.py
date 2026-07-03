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

    # if 'Portico_2D_'+repr(id) not in mdb.models.keys():
    mdb.Model(modelType=STANDARD_EXPLICIT, name='Portico_2D_'+repr(id))

    # if 'Portico_2D_'+repr(id) in mdb.models.keys():
    #     mdb.models.changeKey(fromName='Portico_2D_'+repr(id), toName='Model-1')

    # if 'Model-1' in mdb.models.keys():
    #     mdb.models.changeKey(fromName='Model-1', toName='Portico_2D_'+repr(id))

    model = mdb.models['Portico_2D_'+repr(id)]

    # Parametros Geometricos (SI: metros)
    L = 10.9728  # Vao de 36 ft
    H = 3.048    # Pe-direito de 10 ft


    # Geometria 
    s = model.ConstrainedSketch(name='__profile__', sheetSize=50.0)

    # Desenhando as vigas
    idx_viga = num_pilares
    for j in range(1, pavimentos+1):
        y = j * H
        for i in range(blocos):
            x_start = i * L
            x_end = (i + 1) * L
            
            # Só desenha a viga se o bit não for -1
            if sol[idx_viga] != -1:
                s.Line(point1=(x_start, y), point2=(x_end, y))
                
            idx_viga += 1

    # # Desenhando os pilares
    idx_pilar = 0
    for i in range(blocos+1):
        x = i * L
        for j in range(pavimentos):
            y_start = j * H
            y_end = (j + 1) * H
            
            # So desenha o pilar se o bit correspondente no vetor de solucao for 1
            if sol[idx_pilar] != -1:
                s.Line(point1=(x, y_start), point2=(x, y_end))
                
            idx_pilar += 1

    # Criando a Part
    part = model.Part(
        name='Estrutura',
        dimensionality=TWO_D_PLANAR,
        type=DEFORMABLE_BODY
    )


    part.BaseWire(sketch=s)
    del model.sketches['__profile__']

    # SETS
    # Selecionando arestas pelas coordenadas do ponto medio
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

    # Criando os Sets exigidos
    set_vigas = part.Set(edges=beam_edges, name='Grupo_Vigas')
    set_pilares = part.Set(edges=col_edges, name='Grupo_Pilares')


    # N = len(beam_loc) + len(col_loc)

    # Materiais e sec0es (Perfil W)
    model.Material(name='Aco_Estrutural')
    model.materials['Aco_Estrutural'].Elastic(table=((200E9, 0.3),))
    model.materials['Aco_Estrutural'].Plastic(table=((248.2E6, 0.0),))


    # Alternativas de Perfil para Vigas
    model.IProfile(name='Perfil_W10x12', l=0.1255, h=0.251, b1=0.101, b2=0.101, t1=0.0053, t2=0.0053, t3=0.0048)
    model.IProfile(name='Perfil_W10x15', l=0.1270, h=0.254, b1=0.102, b2=0.102, t1=0.0069, t2=0.0069, t3=0.0058)
    model.IProfile(name='Perfil_W10x17', l=0.1285, h=0.257, b1=0.102, b2=0.102, t1=0.0084, t2=0.0084, t3=0.0061)
    model.IProfile(name='Perfil_W10x19', l=0.1300, h=0.260, b1=0.102, b2=0.102, t1=0.0100, t2=0.0100, t3=0.0064)
    model.IProfile(name='Perfil_W10x22', l=0.1290, h=0.258, b1=0.146, b2=0.146, t1=0.0091, t2=0.0091, t3=0.0061)
    model.IProfile(name='Perfil_W10x26', l=0.1310, h=0.262, b1=0.147, b2=0.147, t1=0.0112, t2=0.0112, t3=0.0066)
    model.IProfile(name='Perfil_W10x30', l=0.1330, h=0.266, b1=0.148, b2=0.148, t1=0.0130, t2=0.0130, t3=0.0076)
    model.IProfile(name='Perfil_HP10x42', l=0.1230, h=0.246, b1=0.256, b2=0.256, t1=0.0107, t2=0.0107, t3=0.0105)
    model.IProfile(name='Perfil_W10x49', l=0.1265, h=0.253, b1=0.254, b2=0.254, t1=0.0142, t2=0.0142, t3=0.0086)
    model.IProfile(name='Perfil_W10x54', l=0.1280, h=0.256, b1=0.255, b2=0.255, t1=0.0156, t2=0.0156, t3=0.0094)
    model.IProfile(name='Perfil_HP10x57', l=0.1270, h=0.254, b1=0.260, b2=0.260, t1=0.0144, t2=0.0144, t3=0.0144)
    model.IProfile(name='Perfil_W10x60', l=0.1300, h=0.260, b1=0.256, b2=0.256, t1=0.0173, t2=0.0173, t3=0.0107)
    model.IProfile(name='Perfil_W10x68', l=0.1320, h=0.264, b1=0.257, b2=0.257, t1=0.0196, t2=0.0196, t3=0.0119)
    model.IProfile(name='Perfil_W10x77', l=0.1345, h=0.269, b1=0.259, b2=0.259, t1=0.0221, t2=0.0221, t3=0.0135)
    model.IProfile(name='Perfil_W10x88', l=0.1375, h=0.275, b1=0.261, b2=0.261, t1=0.0251, t2=0.0251, t3=0.0154)
    model.IProfile(name='Perfil_W10x100', l=0.1410, h=0.282, b1=0.263, b2=0.263, t1=0.0284, t2=0.0284, t3=0.0173)
    model.IProfile(name='Perfil_W10x112', l=0.1445, h=0.289, b1=0.265, b2=0.265, t1=0.0318, t2=0.0318, t3=0.0192)


    # Alternativas de Perfil para Vigas
    model.IProfile(name='Perfil_W21x44', l=0.2625, h=0.525, b1=0.165, b2=0.165, t1=0.0114, t2=0.0114, t3=0.0089)
    model.IProfile(name='Perfil_W21x48', l=0.2620, h=0.524, b1=0.207, b2=0.207, t1=0.0109, t2=0.0109, t3=0.0090)
    model.IProfile(name='Perfil_W21x50', l=0.2645, h=0.529, b1=0.166, b2=0.166, t1=0.0136, t2=0.0136, t3=0.0097)
    model.IProfile(name='Perfil_W21x55', l=0.2640, h=0.528, b1=0.209, b2=0.209, t1=0.0133, t2=0.0133, t3=0.0095)
    model.IProfile(name='Perfil_W21x57', l=0.2675, h=0.535, b1=0.166, b2=0.166, t1=0.0165, t2=0.0165, t3=0.0103)
    model.IProfile(name='Perfil_W21x62', l=0.2665, h=0.533, b1=0.209, b2=0.209, t1=0.0166, t2=0.0166, t3=0.0102)
    model.IProfile(name='Perfil_W21x68', l=0.2685, h=0.537, b1=0.210, b2=0.210, t1=0.0174, t2=0.0174, t3=0.0109)
    model.IProfile(name='Perfil_W21x73', l=0.2695, h=0.539, b1=0.211, b2=0.211, t1=0.0188, t2=0.0188, t3=0.0116)
    model.IProfile(name='Perfil_W21x83', l=0.2720, h=0.544, b1=0.212, b2=0.212, t1=0.0212, t2=0.0212, t3=0.0131)
    model.IProfile(name='Perfil_W21x93', l=0.2745, h=0.549, b1=0.214, b2=0.214, t1=0.0238, t2=0.0238, t3=0.0147)

    model.IProfile(name='Perfil_W24x146', l=0.3140, h=0.628, b1=0.328, b2=0.328, t1=0.0277, t2=0.0277, t3=0.0165)


    # Criando sec0es
    model.BeamSection(
        name='Secao_Pilares',
        integration=DURING_ANALYSIS,
        poissonRatio=0.3,
        profile='Perfil_W10x68',
        material='Aco_Estrutural',
        temperatureVar=LINEAR
    )

    model.BeamSection(
        name='Secao_Vigas',
        integration=DURING_ANALYSIS,
        poissonRatio=0.3,
        profile='Perfil_W21x55',
        material='Aco_Estrutural',
        temperatureVar=LINEAR
    )

    # --------------------------------------------------------------------------
    # Atribuição Dinâmica de Seções para os Pilares baseado no vetor sol
    # --------------------------------------------------------------------------
    idx_pilar = 0
    idx_geometria_existente = 0

    for i in range(blocos + 1):
        for j in range(pavimentos):
            
            bit_ativo = sol[idx_pilar]
            
            # Se o pilar foi removido (-1), pulamos a atribuição dele na geometria
            if bit_ativo != -1:
                
                # Pega a coordenada correta de col_loc usando o contador de elementos reais
                pilar_edge = part.edges.findAt(col_loc[idx_geometria_existente])
                pilar_region = regionToolset.Region(edges=pilar_edge)
                
                perfil_nome = sol[idx_pilar + N]
                
                if bit_ativo == 1:
                    secao_pilar_name = 'Secao_Pilar_' + perfil_nome
                    if secao_pilar_name not in model.sections.keys():
                        model.BeamSection(
                            name=secao_pilar_name,
                            integration=DURING_ANALYSIS,
                            poissonRatio=0.3,
                            profile='Perfil_' + perfil_nome,
                            material='Aco_Estrutural',
                            temperatureVar=LINEAR
                        )
                    
                    part.SectionAssignment(
                        region=pilar_region,
                        sectionName=secao_pilar_name
                    )
                    
                elif bit_ativo == 0:
                    if 'Secao_Pilar_Padrao' not in model.sections.keys():
                        model.BeamSection(
                            name='Secao_Pilar_Padrao',
                            integration=DURING_ANALYSIS,
                            poissonRatio=0.3,
                            profile='Perfil_W10x68',
                            material='Aco_Estrutural',
                            temperatureVar=LINEAR
                        )
                    
                    part.SectionAssignment(
                        region=pilar_region,
                        sectionName='Secao_Pilar_Padrao'
                    )
                
                # Incrementa o ponteiro da lista geométrica já que esse pilar existia
                idx_geometria_existente += 1
                
            idx_pilar += 1


    # --------------------------------------------------------------------------
    # Atribuição Dinâmica de Seções para as Vigas baseado no vetor sol
    # --------------------------------------------------------------------------
    idx_viga = num_pilares
    idx_geometria_viga_existente = 0

    for j in range(1, pavimentos+1):
        for i in range(blocos):
            
            bit_ativo = sol[idx_viga]
            
            if bit_ativo != -1:
                
                # Pega a viga correta usando o contador de vigas reais
                viga_edge = part.edges.findAt(beam_loc[idx_geometria_viga_existente])
                viga_region = regionToolset.Region(edges=viga_edge)
                
                perfil_nome = sol[idx_viga + N]
                
                if bit_ativo == 1:
                    secao_viga_name = 'Secao_Viga_' + perfil_nome
                    if secao_viga_name not in model.sections.keys():
                        model.BeamSection(
                            name=secao_viga_name,
                            integration=DURING_ANALYSIS,
                            poissonRatio=0.3, # Remova eventuais acentos caso insira comentários
                            profile='Perfil_' + perfil_nome,
                            material='Aco_Estrutural',
                            temperatureVar=LINEAR
                        )
                    
                    part.SectionAssignment(
                        region=viga_region,
                        sectionName=secao_viga_name
                    )
                    
                elif bit_ativo == 0:
                    if 'Secao_Viga_Padrao' not in model.sections.keys():
                        model.BeamSection(
                            name='Secao_Viga_Padrao',
                            integration=DURING_ANALYSIS,
                            poissonRatio=0.3,
                            profile='Perfil_W21x55', # Perfil padrão para as vigas
                            material='Aco_Estrutural',
                            temperatureVar=LINEAR
                        )
                    
                    part.SectionAssignment(
                        region=viga_region,
                        sectionName='Secao_Viga_Padrao'
                    )
                
                idx_geometria_viga_existente += 1
                
            idx_viga += 1


    # Orientacao das sec0es de viga
    part.assignBeamSectionOrientation(
        region=set_pilares,
        method=N1_COSINES,
        n1=(0.0, 0.0, -1.0)
    )

    part.assignBeamSectionOrientation(
        region=set_vigas,
        method=N1_COSINES,
        n1=(0.0, 0.0, -1.0)
    )

    # Assembly e Step
    asm = model.rootAssembly

    inst = asm.Instance(
        name='Estrutura_Inst',
        part=part,
        dependent=ON
    )

    model.StaticStep(
        name='Step_Cargas',
        previous='Initial',
        description='Aplicacao das cargas gravitacionais e laterais'
    )

    # Condic0es de contorno
    base_loc = []

    for i in range(blocos+1):
        coordenada = ((i*L, 0.0, 0.0),)   
        base_loc.append(coordenada) 

    base_verts = inst.vertices.findAt(*base_loc)

    region_base = regionToolset.Region(vertices=base_verts)

    model.DisplacementBC(
        name='Apoios_2_Genero_Base',
        createStepName='Initial',
        region=region_base,
        u1=0.0,
        u2=0.0,
        ur3=UNSET
    )

    # Cargas
    # Carga distribuida: 2.8 kip/ft = aproximadamente 40860.89 N/m
    beam_edges_inst = inst.edges.findAt(*beam_loc)

    region_vigas_load = regionToolset.Region(edges=beam_edges_inst)

    model.LineLoad(
        name='Carga_Gravitacional',
        createStepName='Step_Cargas',
        region=region_vigas_load,
        comp2=-40860.89
    )


    # Malha
    # 10 elementos por barra
    all_edges = part.edges[:]

    part.seedEdgeByNumber(
        edges=all_edges,
        number=10,
        constraint=FIXED
    )

    # Elemento de viga 2D linear
    elemTypeBeam = ElemType(
        elemCode=B21,
        elemLibrary=STANDARD
    )

    part.setElementType(
        regions=(all_edges,),
        elemTypes=(elemTypeBeam,)
    )

    part.generateMesh()

    # Atualiza a assembly depois da geracao da malha
    asm.regenerate()

    # Job
    job_name = 'Job_Portico2D'+repr(id)

    # Evita erro se o script for executado mais de uma vez
    if job_name in mdb.jobs.keys():
        del mdb.jobs[job_name]

    job = mdb.Job(
        name=job_name,
        model='Portico_2D_'+repr(id),
        description='Analise Estatica Portico 3 Pavimentoss',
        type=ANALYSIS,
        memory=90,
        memoryUnits=PERCENTAGE,
        numCpus=1,
        numDomains=1
    )

    # Criar arquivos de entrada e submeter analise
    job.submit(consistencyChecking=OFF)
    job.waitForCompletion()

    
    o3 = session.openOdb(name='C:/Users/victo/Desktop/Metodos/temp/Job_Portico2D'+repr(id)+'.odb')
    session.viewports['Viewport: 1'].setValues(displayedObject=o3)
    a = mdb.models['Portico_2D_'+repr(id)].rootAssembly
    session.viewports['Viewport: 1'].setValues(displayedObject=a)
    session.viewports['Viewport: 1'].assemblyDisplay.setValues(optimizationTasks=OFF, geometricRestrictions=OFF, stopConditions=OFF)
    o7 = session.odbs['C:/Users/victo/Desktop/Metodos/temp/Job_Portico2D'+repr(id)+'.odb']
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
    session.printToFile(fileName='C:/Users/victo/Desktop/Metodos/imagens/Job_Portico_2D'+repr(id)+'.png', format=PNG, canvasObjects=(session.viewports['Viewport: 1'], ))



pavimentos = 3
blocos = 2

#--------------------------------------------------------------------------
#Cirar Solucoes
import random
    
num_pilares = (blocos+1)*pavimentos
num_vigas = blocos*pavimentos
N = num_pilares + num_vigas

sol = [0] * (2*N)   

# secoes_vigas = ['W21x44', 'W21x48', 'W21x50', 'W21x55', 'W21x57', 'W21x62', 'W21x68', 'W21x73', 'W21x83', 'W21x93']
# secoes_pilares = ['W10x12', 'W10x15', 'W10x17', 'W10x19', 'W10x22', 'W10x26', 'W10x30', 'HP10x42', 'W10x49', 'W10x54', 'HP10x57', 'W10x60', 'W10x68', 'W10x77', 'W10x88', 'W10x100', 'W10x112']
secoes_vigas = ['W24x146']
secoes_pilares = ['W24x146']


# Preenche a parte dos PILARES
for i in range(num_pilares):
    sol[i] = random.randint(0,1)
    sol[i] = 1
    sol[i+N] = secoes_pilares[random.randint(0,len(secoes_pilares)-1)]

# Preenche a parte das VIGAS
for i in range(num_pilares, N):
    sol[i] = random.randint(0,1)
    sol[i] = 1  
    sol[i+N] = secoes_vigas[random.randint(0,len(secoes_vigas)-1)]

for i in range(num_pilares+1):

    if i == num_pilares:
        gera_estrutura(sol, -1)
        break

    sol_local = sol[:]
    sol_local[i] = -1
    print(sol_local)
    gera_estrutura(sol_local, i)




