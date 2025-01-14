
#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# pylint: disable=invalid-name

"""
Biblioteca Gráfica / Graphics Library.

Desenvolvido por: Lucas Leal e Rafael Almada
Disciplina: Computação Gráfica
Data: 16/08/2021
"""

from os import RTLD_GLOBAL
import time         # Para operações com tempo

import gpu          # Simula os recursos de uma GPU
import numpy as np
import math

class GL:
    """Classe que representa a biblioteca gráfica (Graphics Library)."""

    width = 800   # largura da tela
    height = 600  # altura da tela
    near = 0.01   # plano de corte próximo
    far = 1000    # plano de corte distante

    @staticmethod
    def setup(width, height, near=0.01, far=1000):
        """Definr parametros para câmera de razão de aspecto, plano próximo e distante."""
        GL.width = width
        GL.height = height
        GL.near = near
        GL.far = far
        GL.screen_matrix = np.array([[width / 2,                    0,    0,  width / 2], 
                                     [        0,         - height / 2,    0, height / 2],
                                     [        0,                    0,    1,          0],
                                     [        0,                    0,    0,          1]])
        GL.zBuffer = [ [ None for x in range(GL.width)] for y in range(GL.height) ]
        GL.stack = []
        GL.isThereLight = False
        GL.Light = []
    
    @staticmethod
    def pointsToScreen(points):
        """Função que leva os pontos do espaço 3D para a Tela 2D"""
        # Esta função retorna 'pontos' que é uma lista de numpy arrays com cada ponto, sendo 
        # que cada ponto é uma matriz com x, y, z, w

        # Lista para salvar os pontos
        # print(f"[pointsToScreen] points : {points}")
        pontos = [] 

        # Ordem:
        #   Mundo -> LookAt -> Perspectiva -> divide por w -> screen -> rasteriza   

        # Matriz de Mundo
        GL.world_matrix = np.matmul(np.matmul(GL.translation_matrix, GL.rotation_matrix), GL.scale_matrix)

        if (len(GL.stack) > 1):
            GL.world_matrix = np.matmul(GL.stack[-1], GL.world_matrix)

        GL.stack.append(GL.world_matrix)
                                                                                                                                                                                                                                                                                                                                                                                       
        # Matriz de LookAt -> Camera
        GL.lookAt = np.matmul(np.linalg.inv(GL.orientation_matrix_camera), np.linalg.inv(GL.translation_matrix_camera)) # translation do ponto usado errado

        for i in range(0, len(points), 3):
            p = np.array([[points[i]],
                          [points[i+1]],
                          [points[i+2]],
                          [1]])
            model_to_world = np.matmul(GL.world_matrix, p)

            world_to_cam = np.matmul(GL.lookAt, model_to_world)

            cam_to_perspective = np.matmul(GL.P, world_to_cam)

            # Divide por w
            cam_to_perspective /= cam_to_perspective[3][0]

            perspective_to_screen = np.matmul(GL.screen_matrix, cam_to_perspective)
            
            pontos.append(perspective_to_screen)

        return pontos

    @staticmethod
    def triangleSet(point, colors):
        """Função usada para renderizar TriangleSet."""
        # Nessa função você receberá pontos no parâmetro point, esses pontos são uma lista
        # de pontos x, y, e z sempre na ordem. Assim point[0] é o valor da coordenada x do
        # primeiro ponto, point[1] o valor y do primeiro ponto, point[2] o valor z da
        # coordenada z do primeiro ponto. Já point[3] é a coordenada x do segundo ponto e
        # assim por diante.
        # No TriangleSet os triângulos são informados individualmente, assim os três
        # primeiros pontos definem um triângulo, os três próximos pontos definem um novo
        # triângulo, e assim por diante.
        # O parâmetro colors é um dicionário com os tipos cores possíveis, para o TriangleSet
        # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).
        
        pontos = GL.pointsToScreen(point)

        old_points = []
        for c in range(0, len(point), 3):
            old_points += [point[c:c+3]]

        for tri in range(0, len(pontos), 3):
            # print(f"[TriangleSet] old_points: {old_points[tri:tri+3]}")
            # print(f"[TriangleSet] pontos: {pontos[tri:tri+3]}\n==================================")
            for i in range(GL.width):
                for j in range(GL.height):
                    GL.inside(pontos[tri:tri+3], [i, j], colors, coordenadas=old_points[tri:tri+3])
        
    @staticmethod
    def inside(vertices, ponto, colors, orientation = 2, colorPerVertex = False, cores = None, texture=False, texVertex=None, img = None, coordenadas=None):
        """Função que pinta os pixels delimitados pelos vértices, formando os triângulos"""

        R = int((colors["diffuseColor"][0]+colors["emissiveColor"][0]+colors["specularColor"][0]))
        G = int((colors["diffuseColor"][1]+colors["emissiveColor"][1]+colors["specularColor"][1]))
        B = int((colors["diffuseColor"][2]+colors["emissiveColor"][2]+colors["specularColor"][2]))

        x, y = ponto[0], ponto[1]

        if (orientation % 2 == 0):
            x1, x2, x3 = vertices[0][0][0], vertices[1][0][0], vertices[2][0][0]
            y1, y2, y3 = vertices[0][1][0], vertices[1][1][0], vertices[2][1][0]
            if (len(vertices[0]) > 2):
                z1, z2, z3 = vertices[0][2][0], vertices[1][2][0], vertices[2][2][0]
        else:
            # Inverte a ordem dos pontos
            x1, x2, x3 = vertices[2][0][0], vertices[1][0][0], vertices[0][0][0]
            y1, y2, y3 = vertices[2][1][0], vertices[1][1][0], vertices[0][1][0]
            if (len(vertices[0]) > 3):
                z1, z2, z3 = vertices[2][2][0], vertices[1][2][0], vertices[0][2][0]

        # P2 - P1
        dX0 = x2 - x1
        dY0 = y2 - y1

        # P2 - P3
        dX1 = x3 - x2
        dY1 = y3 - y2

        # P3 - P1
        dX2 = x1 - x3
        dY2 = y1 - y3

        L0 = (x - x1)*dY0 - (y - y1)*dX0
        L1 = (x - x2)*dY1 - (y - y2)*dX1
        L2 = (x - x3)*dY2 - (y - y3)*dX2

        if (L0 >= 0 and L1 >= 0 and L2 >= 0):
            alpha = (-(x - x2)*(y3 - y2) + (y - y2)*(x3 - x2)) / (-(x1 - x2)*(y3 - y2) + (y1 - y2)*(x3 - x2))
                
            beta  = (-(x - x3)*(y1 - y3) + (y - y3)*(x1 - x3)) / (-(x2 - x3)*(y1 - y3) + (y2 - y3)*(x1 - x3))
            
            gamma = 1 - (alpha + beta) # soma = alpha + beta + gamma # Deve ser sempre igual a 1
            # print(f"[INSIDE - ANTES] orientation = {orientation}")
            if texture:
                # Pintar a textura
                tex_x = (texVertex[0][0]*alpha + texVertex[1][0]*beta + texVertex[2][0]*gamma)*img.shape[0]
                tex_y = (texVertex[0][1]*alpha + texVertex[1][1]*beta + texVertex[2][1]*gamma)*img.shape[1]
                # print(f"new_x = {new_x} e new_y = {new_y}")
                
                R, G, B, a = img[int(-tex_y)][int(tex_x)]

            elif colorPerVertex and cores != None:
                R = (cores[0][0] * alpha + cores[1][0] * beta + cores[2][0] * gamma) * 255
                G = (cores[0][1] * alpha + cores[1][1] * beta + cores[2][1] * gamma) * 255
                B = (cores[0][2] * alpha + cores[1][2] * beta + cores[2][2] * gamma) * 255
                
            elif GL.isThereLight:
                # print(f"[INSIDE - DEPOIS] orientation = {orientation}")
                # Definindo propriedades do material
                O_d = np.array([colors["diffuseColor"][0], colors["diffuseColor"][1], colors["diffuseColor"][2]])
                O_e = np.array([colors["emissiveColor"][0], colors["emissiveColor"][1], colors["emissiveColor"][2]])
                O_s = np.array([colors["specularColor"][0], colors["specularColor"][1], colors["specularColor"][2]])
                s = colors["shininess"] # Valor do shininess
                # O_a = colors["ambientIntensity"]
                
                # Definindo o vetor normal a partir de 2 vetores do plano
                # print(f"[INSIDE] vertices : {vertices}")
                # print(f"[INSIDE] coordenadas : {coordenadas}")
                if (orientation % 2 == 0):
                    o_x1, o_x2, o_x3 = coordenadas[0][0], coordenadas[1][0], coordenadas[2][0]
                    o_y1, o_y2, o_y3 = coordenadas[0][1], coordenadas[1][1], coordenadas[2][1]
                    o_z1, o_z2, o_z3 = coordenadas[0][2], coordenadas[1][2], coordenadas[2][2]
                else:
                    # Inverte a ordem dos pontos
                    # print("isso nunca vai para o terminal")
                    o_x1, o_x2, o_x3 = coordenadas[2][0], coordenadas[1][0], coordenadas[0][0]
                    o_y1, o_y2, o_y3 = coordenadas[2][1], coordenadas[1][1], coordenadas[0][1]
                    o_z1, o_z2, o_z3 = coordenadas[2][2], coordenadas[1][2], coordenadas[0][2]
                
                p = np.array([o_x2 - o_x1, o_y2 - o_y1, o_z2 - o_z1]) # Vetor p do plano
                q = np.array([o_x3 - o_x1, o_y3 - o_y1, o_z3 - o_z1]) # Vetor q do plano
                N = np.cross(p, q)# Calculo da Normal (Produto vetorial)
                norm = np.linalg.norm(N)
                N = np.divide(N, norm)
                v = np.array([0.0, 0.0, 1.0])
                
                sum_luz = np.array([0.0, 0.0, 0.0])
                for light in GL.Light:
                    # Calculando valores da luz com material
                    ambient = O_d * light[1] * 0.2 # * O_a no lugar de 0.2
                    diffuse = O_d * light[3] * np.dot(N, light[0])

                    spec_1 = np.divide(np.add(light[0], v), np.linalg.norm(np.add(light[0], v)))
                    spec_2 = np.dot(N, spec_1)
                    specular = O_s * light[3] * spec_2**(s*128)
                    sum_luz += light[2] * (ambient + diffuse + specular)

                I_rgb = O_e + sum_luz
                
                R = I_rgb[0]*255
                G = I_rgb[1]*255
                B = I_rgb[2]*255
                if (R > 255):
                    R = 255
                elif (R < 0 or np.isnan(R)):
                    R = 0
                if (B > 255):
                    B = 255
                elif (B < 0 or np.isnan(B)):
                    B = 0
                if (G > 255):
                    G = 255
                elif (G < 0 or np.isnan(G)):
                    G = 0

            Z = 1 / (alpha*(1/z1) + beta*(1/z2) + gamma*(1/z3))
            if (GL.zBuffer[int(y)][int(x)] != None):
                if (Z < GL.zBuffer[int(y)][int(x)]):
                    GL.zBuffer[int(y)][int(x)] = Z
                    gpu.GPU.draw_pixels([int(x), int(y)], gpu.GPU.RGB8, [R, G, B])  # altera pixel
            else:
                GL.zBuffer[int(y)][int(x)] = Z
                gpu.GPU.draw_pixels([int(x), int(y)], gpu.GPU.RGB8, [R, G, B])  # altera pixel

            

    @staticmethod
    def viewpoint(position, orientation, fieldOfView):
        """Função usada para renderizar (na verdade coletar os dados) de Viewpoint."""
        # Na função de viewpoint você receberá a posição, orientação e campo de visão da
        # câmera virtual. Use esses dados para poder calcular e criar a matriz de projeção
        # perspectiva para poder aplicar nos pontos dos objetos geométricos.

        # Calculando o FOV em y. 
        fovy = 2*math.atan(math.tan(fieldOfView/2)*GL.height/math.sqrt(GL.height**2 + GL.width**2))
        
        # parametrizando a visao perspectiva
        top    = GL.near*math.tan(fovy)
        right  = top*(GL.width/GL.height)

        GL.P = np.array([[GL.near/right, 0, 0, 0],
                         [0, GL.near/top, 0, 0],
                         [0, 0, -((GL.far+GL.near)/(GL.far-GL.near)), -((2*GL.far*GL.near)/(GL.far-GL.near))],
                         [0,0,-1,0]])

        GL.translation_matrix_camera= np.array([[1, 0, 0, position[0]],
                                               [0, 1, 0, position[1]],
                                               [0, 0, 1, position[2]],
                                               [0, 0, 0, 1]])
        if orientation:
            if (orientation[0] > 0):
                # Rotação em x
                GL.orientation_matrix_camera = np.array([[1,0,0,0],
                                              [0,math.cos(orientation[3]),-math.sin(orientation[3]),0],
                                              [0,math.sin(orientation[3]),math.cos(orientation[3]),0],
                                              [0,0,0,1]])
            elif (orientation[1] > 0):
                # Rotação em y
                GL.orientation_matrix_camera = np.array([[math.cos(orientation[3]), 0, math.sin(orientation[3]), 0],
                                               [0, 1, 0, 0],
                                               [-math.sin(orientation[3]), 0, math.cos(orientation[3]), 0],
                                               [0, 0, 0, 1]])
            else:
                # Rotação em z     
                GL.orientation_matrix_camera = np.array([[math.cos(orientation[3]),-math.sin(orientation[3]),0,0],
                                               [math.sin(orientation[3]), math.cos(orientation[3]), 0, 0],
                                               [0, 0, 1, 0],
                                               [0, 0, 0, 1]])

    @staticmethod
    def transform_in(translation, scale, rotation):
        """Função usada para renderizar (na verdade coletar os dados) de Transform."""
        # A função transform_in será chamada quando se entrar em um nó X3D do tipo Transform
        # do grafo de cena. Os valores passados são a escala em um vetor [x, y, z]
        # indicando a escala em cada direção, a translação [x, y, z] nas respectivas
        # coordenadas e finalmente a rotação por [x, y, z, t] sendo definida pela rotação
        # do objeto ao redor do eixo x, y, z por t radianos, seguindo a regra da mão direita.
        # Quando se entrar em um nó transform se deverá salvar a matriz de transformação dos
        # modelos do mundo em alguma estrutura de pilha.
        # scale rot transl

        if translation:
            #print("aaaaaaaaaaaaaaaaaaaaaaaaa",translation[0],translation[1],translation[2],)
            GL.translation_matrix = np.array([[1, 0, 0, translation[0]],
                                              [0, 1, 0, translation[1]],
                                              [0, 0, 1, translation[2]],
                                              [0, 0, 0, 1]])
            
        if rotation:
            if (rotation[0] > 0):
                # Rotação em x
                GL.rotation_matrix = np.array([[1,0,0,0],
                                              [0,math.cos(rotation[3]),-math.sin(rotation[3]),0],
                                              [0,math.sin(rotation[3]),math.cos(rotation[3]),0],
                                              [0,0,0,1]])
            elif (rotation[1] > 0):
                # Rotação em y
                GL.rotation_matrix = np.array([[math.cos(rotation[3]), 0, math.sin(rotation[3]), 0],
                                               [0, 1, 0, 0],
                                               [-math.sin(rotation[3]), 0, math.cos(rotation[3]), 0],
                                               [0, 0, 0, 1]])
            else:
                # Rotação em z     
                GL.rotation_matrix = np.array([[math.cos(rotation[3]),-math.sin(rotation[3]),0,0],
                                               [math.sin(rotation[3]), math.cos(rotation[3]), 0, 0],
                                               [0, 0, 1, 0],
                                               [0, 0, 0, 1]])

        if scale:
            GL.scale_matrix = np.array([[scale[0],0,0,0],
                                        [0,scale[1],0,0],
                                        [0,0,scale[2],0],
                                        [0,0,0,1]])
            
    @staticmethod
    def transform_out():
        """Função usada para renderizar (na verdade coletar os dados) de Transform."""
        # A função transform_out será chamada quando se sair em um nó X3D do tipo Transform do
        # grafo de cena. Não são passados valores, porém quando se sai de um nó transform se
        # deverá recuperar a matriz de transformação dos modelos do mundo da estrutura de
        # pilha implementada.
        if (len(GL.stack) > 0):
            GL.stack.pop()

    @staticmethod
    def triangleStripSet(point, stripCount, colors):
        """Função usada para renderizar TriangleStripSet."""
        # A função triangleStripSet é usada para desenhar tiras de triângulos interconectados,
        # você receberá as coordenadas dos pontos no parâmetro point, esses pontos são uma
        # lista de pontos x, y, e z sempre na ordem. Assim point[0] é o valor da coordenada x
        # do primeiro ponto, point[1] o valor y do primeiro ponto, point[2] o valor z da
        # coordenada z do primeiro ponto. Já point[3] é a coordenada x do segundo ponto e assim
        # por diante. No TriangleStripSet a quantidade de vértices a serem usados é informado
        # em uma lista chamada stripCount (perceba que é uma lista). Ligue os vértices na ordem,
        # primeiro triângulo será com os vértices 0, 1 e 2, depois serão os vértices 1, 2 e 3,
        # depois 2, 3 e 4, e assim por diante. Cuidado com a orientação dos vértices, ou seja,
        # todos no sentido horário ou todos no sentido anti-horário, conforme especificado.

        pontos = GL.pointsToScreen(point)

        old_points = []
        for c in range(0, len(point), 3):
            old_points += [point[c:c+3]]

        orientation = 2
        for i in range(stripCount[0]):
            #borders = [min(pontos[i][0][0], pontos[i+1][0][0], pontos[i+2][0][0]), max(pontos[i][0][0], pontos[i+1][0][0], pontos[i+2][0][0]),
            #           min(pontos[i][1][0], pontos[i+1][1][0], pontos[i+2][1][0]), max(pontos[i][1][0], pontos[i+1][1][0], pontos[i+2][1][0])]
            for x in range(GL.width):#math.floor(borders[0]), math.ceil(borders[1])):
                for y in range(GL.height):#math.floor(borders[2]), math.ceil(borders[3])):
                    if (i+3 > len(pontos)):
                        break
                    else:
                        GL.inside(pontos[i:i+3], [x, y], colors, orientation=orientation, coordenadas=old_points[i:i+3])
            orientation += 1

    @staticmethod
    def indexedTriangleStripSet(point, index, colors):
        """Função usada para renderizar IndexedTriangleStripSet."""
        # A função indexedTriangleStripSet é usada para desenhar tiras de triângulos
        # interconectados, você receberá as coordenadas dos pontos no parâmetro point, esses
        # pontos são uma lista de pontos x, y, e z sempre na ordem. Assim point[0] é o valor
        # da coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto, point[2]
        # o valor z da coordenada z do primeiro ponto. Já point[3] é a coordenada x do
        # segundo ponto e assim por diante. No IndexedTriangleStripSet uma lista informando
        # como conectar os vértices é informada em index, o valor -1 indica que a lista
        # acabou. A ordem de conexão será de 3 em 3 pulando um índice. Por exemplo: o
        # primeiro triângulo será com os vértices 0, 1 e 2, depois serão os vértices 1, 2 e 3,
        # depois 2, 3 e 4, e assim por diante. Cuidado com a orientação dos vértices, ou seja,
        # todos no sentido horário ou todos no sentido anti-horário, conforme especificado.

        pontos = GL.pointsToScreen(point)
        old_points = []
        for c in range(0, len(point), 3):
            old_points += [point[c:c+3]]
        orientation = 1

        for i in index:
            if (index[i+2] < 0):
                break
            orientation += 1
            borders = [min(pontos[i][0][0], pontos[i+1][0][0], pontos[i+2][0][0]), max(pontos[i][0][0], pontos[i+1][0][0], pontos[i+2][0][0]),
                       min(pontos[i][1][0], pontos[i+1][1][0], pontos[i+2][1][0]), max(pontos[i][1][0], pontos[i+1][1][0], pontos[i+2][1][0])]
            for x in range(math.floor(borders[0]), math.ceil(borders[1])):
                for y in range(math.floor(borders[2]), math.ceil(borders[3])):
                    # print("==================================================")
                    # print(f"pontos[{i}:{i+3}] = {pontos[i:i+3]}")
                    # print(f"old_points[{i}:{i+3}] = {old_points[i:i+3]}")
                    GL.inside(pontos[i:i+3], [x, y], colors, orientation= orientation, coordenadas=old_points[i:i+3])

    @staticmethod
    def box(size, colors):
        """Função usada para renderizar Boxes."""
        # A função box é usada para desenhar paralelepípedos na cena. O Box é centrada no
        # (0, 0, 0) no sistema de coordenadas local e alinhado com os eixos de coordenadas
        # locais. O argumento size especifica as extensões da caixa ao longo dos eixos X, Y
        # e Z, respectivamente, e cada valor do tamanho deve ser maior que zero. Para desenhar
        # essa caixa você vai provavelmente querer tesselar ela em triângulos, para isso
        # encontre os vértices e defina os triângulos.
        
        # Escrevendo os pontos em uma lista
        # (Escalonamos os pontos para ter uma imagem maior e mais fácil de visualizar)
        points = [-size[0], -size[1],  size[2], # P1 = (-1, -1, 1)
                   size[0], -size[1],  size[2], # P2 = (1, -1, 1)
                   size[0],  size[1],  size[2], # P3 = (1, 1, 1)
                  -size[0],  size[1],  size[2], # P4 = (-1, 1, 1)
                  -size[0],  size[1], -size[2], # P5 = (-1, 1, -1)
                   size[0],  size[1], -size[2], # P6 = (1, 1, -1)
                  -size[0], -size[1], -size[2], # P7 = (-1, -1, -1)
                   size[0], -size[1], -size[2]] # P8 = (1, -1, -1)

        pontos = GL.pointsToScreen(points)

        old_coords = [[points[0], points[1], points[2]], # 1  -> frontal
                      [points[2], points[3], points[0]], # 2  -> frontal
                      [points[3], points[2], points[5]], # 7  -> superior
                      [points[5], points[4], points[3]], # 8  -> superior
                      [points[1], points[7], points[5]], # 11 -> direita
                      [points[5], points[2], points[1]],  # 12 -> direita
                      [points[4], points[5], points[6]], # 9 -> Nao aparece
                      [points[5], points[6], points[7]], # 10 -> Nao aparece
                      [points[6], points[4], points[0]], # 3 -> Nao aparece
                      [points[4], points[3], points[0]], # 4 -> Nao aparece
                      [points[7], points[1], points[0]], # 5 -> Nao aparece
                      [points[7], points[6], points[0]] # 6 -> Nao Aparece
        ]

        valid_triangles = [[pontos[0], pontos[1], pontos[2]], # 1  -> frontal
                            [pontos[2], pontos[3], pontos[0]], # 2  -> frontal
                            [pontos[3], pontos[2], pontos[5]], # 7  -> superior
                            [pontos[5], pontos[4], pontos[3]], # 8  -> superior
                            [pontos[1], pontos[7], pontos[5]], # 11 -> direita
                            [pontos[5], pontos[2], pontos[1]],  # 12 -> direita
                            [pontos[4], pontos[5], pontos[6]], # 9 -> Nao aparece
                            [pontos[5], pontos[6], pontos[7]], # 10 -> Nao aparece
                            [pontos[6], pontos[4], pontos[0]], # 3 -> Nao aparece
                            [pontos[4], pontos[3], pontos[0]], # 4 -> Nao aparece
                            [pontos[7], pontos[1], pontos[0]], # 5 -> Nao aparece
                            [pontos[7], pontos[6], pontos[0]] # 6 -> Nao Aparece
                          ]       
        orientation = 2
        for triangulo in range(0, len(valid_triangles), 2):
            for x in range(GL.width):
                for y in range(GL.height):
                    GL.inside(valid_triangles[triangulo], [x, y], colors, orientation, coordenadas=old_coords)
                    GL.inside(valid_triangles[triangulo+1], [x, y], colors, orientation, coordenadas=old_coords)
            orientation += 1

    @staticmethod
    def indexedFaceSet(coord, coordIndex, colorPerVertex, color, colorIndex,
                       texCoord, texCoordIndex, colors, current_texture):
        """Função usada para renderizar IndexedFaceSet."""
        # A função indexedFaceSet é usada para desenhar malhas de triângulos. Ela funciona de
        # forma muito simular a IndexedTriangleStripSet porém com mais recursos.
        # Você receberá as coordenadas dos pontos no parâmetro cord, esses
        # pontos são uma lista de pontos x, y, e z sempre na ordem. Assim coord[0] é o valor
        # da coordenada x do primeiro ponto, coord[1] o valor y do primeiro ponto, coord[2]
        # o valor z da coordenada z do primeiro ponto. Já coord[3] é a coordenada x do
        # segundo ponto e assim por diante. No IndexedFaceSet uma lista de vértices é informada
        # em coordIndex, o valor -1 indica que a lista acabou.
        # A ordem de conexão será de 3 em 3 pulando um índice. Por exemplo: o
        # primeiro triângulo será com os vértices 0, 1 e 2, depois serão os vértices 1, 2 e 3,
        # depois 2, 3 e 4, e assim por diante. 
        # Adicionalmente essa implementação do IndexedFace aceita cores por vértices, assim
        # se a flag colorPerVertex estiver habilitada, os vértices também possuirão cores
        # que servem para definir a cor interna dos poligonos, para isso faça um cálculo
        # baricêntrico de que cor deverá ter aquela posição. Da mesma forma se pode definir uma
        # textura para o poligono, para isso, use as coordenadas de textura e depois aplique a
        # cor da textura conforme a posição do mapeamento. Dentro da classe GPU já está
        # implementadado um método para a leitura de imagens.
        if current_texture:
            image = gpu.GPU.load_texture(current_texture[0])

        # Dividir os pontos em vertices
        vertex = GL.pointsToScreen(coord)
        triangle, old_vertex, trueColor = [], [], []
        
        if colorPerVertex and color and colorIndex:
            cores = []
            for c in range(0, len(color), 3): 
                cores.append([color[c], color[c+1], color[c+2]])

            for vert, c in zip(coordIndex, colorIndex):
                if vert < 0 or c < 0:
                    borders = [min(triangle[0][0][0], triangle[1][0][0], triangle[2][0][0]), max(triangle[0][0][0], triangle[1][0][0], triangle[2][0][0]),
                               min(triangle[0][1][0], triangle[1][1][0], triangle[2][1][0]), max(triangle[0][1][0], triangle[1][1][0], triangle[2][1][0])]
                    for x in range(math.floor(borders[0]), math.ceil(borders[1])):
                        for y in range(math.floor(borders[2]), math.ceil(borders[3])):
                            GL.inside(triangle, [x, y], colors, colorPerVertex = colorPerVertex, cores = trueColor, coordenadas=old_vertex)
                    triangle, trueColor, old_vertex = [], [], []
                    continue
                trueColor.append(cores[c])
                triangle.append(vertex[vert])
                old_vertex.append(coord[vert:vert+3])

        elif (texCoord and texCoordIndex):
            texturePoints = []
            for p in range(0, len(texCoord), 2):
                texturePoints.append([texCoord[p], texCoord[p+1]])
            texTriangle = []
            for vert, tex in zip(coordIndex, texCoordIndex):
                if vert < 0:
                    # Encontrando as bordas das partes a serem pintadas para fins de otimização
                    borders = [min(triangle[0][0][0], triangle[1][0][0], triangle[2][0][0]), max(triangle[0][0][0], triangle[1][0][0], triangle[2][0][0]),
                               min(triangle[0][1][0], triangle[1][1][0], triangle[2][1][0]), max(triangle[0][1][0], triangle[1][1][0], triangle[2][1][0])]
                    for x in range(math.floor(borders[0]), math.ceil(borders[1])):
                        for y in range(math.floor(borders[2]), math.ceil(borders[3])):
                            GL.inside(triangle, [x, y], colors, texture=True, texVertex=texTriangle, img=image, coordenadas=old_vertex)
                    triangle, texTriangle, old_vertex = [], [], []
                    continue
                triangle.append(vertex[vert])
                texTriangle.append(texturePoints[tex])
                old_vertex.append(coord[vert:vert+3])
        else:
            for vert in coordIndex:
                if vert < 0:
                    borders = [min(triangle[0][0][0], triangle[1][0][0], triangle[2][0][0]), max(triangle[0][0][0], triangle[1][0][0], triangle[2][0][0]),
                               min(triangle[0][1][0], triangle[1][1][0], triangle[2][1][0]), max(triangle[0][1][0], triangle[1][1][0], triangle[2][1][0])]
                    for x in range(math.floor(borders[0]), math.ceil(borders[1])):
                        for y in range(math.floor(borders[2]), math.ceil(borders[3])):
                            GL.inside(triangle, [x, y], colors, coordenadas=old_vertex)
                    triangle, old_vertex = [], []
                    continue
                triangle.append(vertex[vert])
                old_vertex.append(coord[vert:vert+3])
                
                # print(f"old_vertex = {old_vertex}")

        # Exemplo de desenho de um pixel branco na coordenada 10, 10
        # gpu.GPU.draw_pixels([10, 10], gpu.GPU.RGB8, [255, 255, 255])  # altera pixel

    @staticmethod
    def sphere(radius, colors):
        """Função usada para renderizar Esferas."""
        # A função sphere é usada para desenhar esferas na cena. O esfera é centrada no
        # (0, 0, 0) no sistema de coordenadas local. O argumento radius especifica o
        # raio da esfera que está sendo criada. Para desenhar essa esfera você vai
        # precisar tesselar ela em triângulos, para isso encontre os vértices e defina
        # os triângulos.
        points = []
        
        # Step de graus a serem calculados
        step = math.pi/81
        degrees_phi = np.arange(0, math.pi+step, step) # Variação dos graus 'phi'
        degrees_theta = np.arange(0, 2*math.pi+step, step) # Variação dos graus 'theta'

        sizes = [0]
        # Calculando coordenadas dos pontos a partir do raio e dos graus 'phi' e 'theta'
        for phi in degrees_phi:
            faixa= []
            x_points, y_points, z_points = [], [], []
            for theta in degrees_theta:
                # Calcula o ponto (x, y, z)
                x = radius * math.cos(theta) * math.sin(phi)
                y = radius * math.cos(phi)
                z = radius * math.sin(theta) * math.sin(phi)
                if (phi in [0.0, math.pi]):
                    x_points += [x]
                    y_points += [y]
                    z_points += [z]
                faixa += [x, y, z]
            if (phi in [0.0, math.pi]):
                # Achando o ponto médio para fazer o nó da ponta da esfera
                x = sum(x_points)/(len(x_points))
                y = sum(y_points)/(len(y_points))
                z = sum(z_points)/(len(z_points))
                faixa = [x, y, z]
            points += faixa
            sizes += [int(len(points)/3)] # Referencia do número de pontos por faixa

        # Transformando as coordenadas para pontos na tela
        coords = []
        coords.append(GL.pointsToScreen(points))

        # Ordenando pontos na tela por faixa de valores em y
        vertex_coords = []
        for i in range(1, len(sizes)):
            vertex_coords.append(coords[0][sizes[i-1]:sizes[i]])

        # [[[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]], [...], [...]]
        c, old_coords = [], []
        for p in range(0, len(points), 3):
            c.append([points[p], points[p+1], points[p+2]])
        for i in range(1, len(sizes)):
            old_coords.append(c[sizes[i-1]:sizes[i]])
        
        # Desenhando os triângulos na tela
        for fx in range(len(vertex_coords) - 1):
            if (len(vertex_coords[fx]) == 1 and fx == 0):
                # Caso da ponta inicial da esfera
                for p in range(len(vertex_coords[fx+1])):
                    # Ordem de conexão dos vértices do triângulo
                    v = [vertex_coords[fx+1][p-1], vertex_coords[fx][0], vertex_coords[fx+1][p]]
                    c = [old_coords[fx+1][p-1], old_coords[fx][0], old_coords[fx+1][p]]

                    # Encontrando as bordas das partes a serem pintadas para fins de otimização
                    borders = [min(v[0][0][0], v[1][0][0], v[2][0][0]), max(v[0][0][0], v[1][0][0], v[2][0][0]),
                               min(v[0][1][0], v[1][1][0], v[2][1][0]), max(v[0][1][0], v[1][1][0], v[2][1][0])]
                    
                    # Desenhando triângulos na tela
                    for x in range(round(borders[0]), round(borders[1])):
                        for y in range(round(borders[2]), round(borders[3])):
                            GL.inside(v, (x, y), colors, coordenadas=c)
                continue
            elif (len(vertex_coords[fx+1]) == 1 and fx == len(vertex_coords) - 2):
                # Caso da ponta final da esfera
                for p in range(len(vertex_coords[fx])):
                    # Ordem de conexão dos vértices do triângulo
                    v = [vertex_coords[fx][p], vertex_coords[fx][p-1], vertex_coords[fx+1][0]]
                    c = [old_coords[fx][p], old_coords[fx][p-1], old_coords[fx+1][0]]

                    # Encontrando as bordas das partes a serem pintadas para fins de otimização
                    borders = [min(v[0][0][0], v[1][0][0], v[2][0][0]), max(v[0][0][0], v[1][0][0], v[2][0][0]),
                               min(v[0][1][0], v[1][1][0], v[2][1][0]), max(v[0][1][0], v[1][1][0], v[2][1][0])]
                    
                    # Desenhando triângulos na tela
                    for x in range(round(borders[0]), round(borders[1])):
                        for y in range(round(borders[2]), round(borders[3])):
                            GL.inside(v, (x, y), colors, coordenadas=c)
                continue
            for p in range(len(vertex_coords[fx])):
                # Faixas do meio

                # Ordem de conexão dos vértices do triângulo
                v1 = [vertex_coords[fx][p], vertex_coords[fx+1][p], vertex_coords[fx+1][p-1]]
                v2 = [vertex_coords[fx][p], vertex_coords[fx+1][p-1], vertex_coords[fx][p-1]]

                c1 = [old_coords[fx][p], old_coords[fx+1][p], old_coords[fx+1][p-1]]
                c2 = [old_coords[fx][p], old_coords[fx+1][p-1], old_coords[fx][p-1]]

                # Encontrando as bordas das partes a serem pintadas para fins de otimização
                borders = [min(v1[0][0][0], v2[0][0][0], v1[1][0][0], v2[1][0][0], v1[2][0][0], v2[2][0][0]), 
                           max(v1[0][0][0], v2[0][0][0], v1[1][0][0], v2[1][0][0], v1[2][0][0], v2[2][0][0]),
                           min(v1[0][1][0], v2[0][1][0], v1[1][1][0], v2[1][1][0], v1[2][1][0], v2[2][1][0]), 
                           max(v1[0][1][0], v2[0][1][0], v1[1][1][0], v2[1][1][0], v1[2][1][0], v2[2][1][0])]

                # Desenhando triângulos na tela
                for x in range(math.floor(borders[0]), math.ceil(borders[1])):
                    for y in range(math.floor(borders[2]), math.ceil(borders[3])):
                        GL.inside(v1, (x, y), colors, coordenadas=c1)
                        GL.inside(v2, (x, y), colors, coordenadas=c2)

    @staticmethod
    def navigationInfo(headlight):
        """Características físicas do avatar do visualizador e do modelo de visualização."""
        # O campo do headlight especifica se um navegador deve acender um luz direcional que
        # sempre aponta na direção que o usuário está olhando. Definir este campo como TRUE
        # faz com que o visualizador forneça sempre uma luz do ponto de vista do usuário.
        # A luz headlight deve ser direcional, ter intensidade = 1, cor = (1 1 1),
        # ambientIntensity = 0,0 e direção = (0 0 −1).

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("NavigationInfo : headlight = {0}".format(headlight)) # imprime no terminal
        if headlight:
            GL.directionalLight(0.0, [1.0, 1.0, 1.0], 1, [0.0, 0.0, -1.0])

    @staticmethod
    def directionalLight(ambientIntensity, color, intensity, direction):
        """Luz direcional ou paralela."""
        # Define uma fonte de luz direcional que ilumina ao longo de raios paralelos
        # em um determinado vetor tridimensional. Possui os campos básicos ambientIntensity,
        # cor, intensidade. O campo de direção especifica o vetor de direção da iluminação
        # que emana da fonte de luz no sistema de coordenadas local. A luz é emitida ao
        # longo de raios paralelos de uma distância infinita.

        GL.isThereLight = True # Flag caso haja luz
        # Salvando dados da luz em variável global
        # GL.Light -> Lista com listas de dados das luzes declaradas no x3d
        # Ordem: [direcao da luz, intensidade ambiente da luz (I_ia), cor da luz (I_Lrgb), intensidade da luz (I_i)]
        GL.Light += [[np.array([-direction[0], -direction[1], -direction[2]]), ambientIntensity, np.array(color), intensity]]


    @staticmethod
    def pointLight(ambientIntensity, color, intensity, location):
        """Luz pontual."""
        # Fonte de luz pontual em um local 3D no sistema de coordenadas local. Uma fonte
        # de luz pontual emite luz igualmente em todas as direções; ou seja, é omnidirecional.
        # Possui os campos básicos ambientIntensity, cor, intensidade. Um nó PointLight ilumina
        # a geometria em um raio de sua localização. O campo do raio deve ser maior ou igual a
        # zero. A iluminação do nó PointLight diminui com a distância especificada.

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("PointLight : ambientIntensity = {0}".format(ambientIntensity))
        print("PointLight : color = {0}".format(color)) # imprime no terminal
        print("PointLight : intensity = {0}".format(intensity)) # imprime no terminal
        print("PointLight : location = {0}".format(location)) # imprime no terminal

    @staticmethod
    def fog(visibilityRange, color):
        """Névoa."""
        # O nó Fog fornece uma maneira de simular efeitos atmosféricos combinando objetos
        # com a cor especificada pelo campo de cores com base nas distâncias dos
        # vários objetos ao visualizador. A visibilidadeRange especifica a distância no
        # sistema de coordenadas local na qual os objetos são totalmente obscurecidos
        # pela névoa. Os objetos localizados fora de visibilityRange do visualizador são
        # desenhados com uma cor de cor constante. Objetos muito próximos do visualizador
        # são muito pouco misturados com a cor do nevoeiro.

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("Fog : color = {0}".format(color)) # imprime no terminal
        print("Fog : visibilityRange = {0}".format(visibilityRange))

    @staticmethod
    def timeSensor(cycleInterval, loop):
        """Gera eventos conforme o tempo passa."""
        # Os nós TimeSensor podem ser usados para muitas finalidades, incluindo:
        # Condução de simulações e animações contínuas; Controlar atividades periódicas;
        # iniciar eventos de ocorrência única, como um despertador;
        # Se, no final de um ciclo, o valor do loop for FALSE, a execução é encerrada.
        # Por outro lado, se o loop for TRUE no final de um ciclo, um nó dependente do
        # tempo continua a execução no próximo ciclo. O ciclo de um nó TimeSensor dura
        # cycleInterval segundos. O valor de cycleInterval deve ser maior que zero.

        # Deve retornar a fração de tempo passada em fraction_changed

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("TimeSensor : cycleInterval = {0}".format(cycleInterval)) # imprime no terminal
        print("TimeSensor : loop = {0}".format(loop))

        # Esse método já está implementado para os alunos como exemplo
        epoch = time.time()  # time in seconds since the epoch as a floating point number.
        fraction_changed = (epoch % cycleInterval) / cycleInterval

        return fraction_changed

    @staticmethod
    def splinePositionInterpolator(set_fraction, key, keyValue, closed):
        """Interpola não linearmente entre uma lista de vetores 3D."""
        # Interpola não linearmente entre uma lista de vetores 3D. O campo keyValue possui
        # uma lista com os valores a serem interpolados, key possui uma lista respectiva de chaves
        # dos valores em keyValue, a fração a ser interpolada vem de set_fraction que varia de
        # zero a um. O campo keyValue deve conter exatamente tantos vetores 3D quanto os
        # quadros-chave no key. O campo closed especifica se o interpolador deve tratar a malha
        # como fechada, com uma transições da última chave para a primeira chave. Se os keyValues
        # na primeira e na última chave não forem idênticos, o campo closed será ignorado.

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("SplinePositionInterpolator : set_fraction = {0}".format(set_fraction))
        print("SplinePositionInterpolator : key = {0}".format(key)) # imprime no terminal
        print("SplinePositionInterpolator : keyValue = {0}".format(keyValue))
        print("SplinePositionInterpolator : closed = {0}".format(closed))

        # Abaixo está só um exemplo de como os dados podem ser calculados e transferidos
        # value_changed = [0.0, 0.0, 0.0]
        t_antes = []
        t_depois = []
        # 0.5
        # [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
        new_keyValue = []
        for k in range(0, len(keyValue), 3):
            new_keyValue += [np.array([keyValue[k], keyValue[k+1], keyValue[k+2]])] # Coordenadas ao longo do tempo
        
        if (set_fraction < key[1] or set_fraction > key[-2]):
            if not closed:
                if (set_fraction < key[1]):
                    t_antes += [key[0], 0]
                    t_depois += [key[1], 1]
                else:
                    t_antes += [key[-2], -2]
                    t_depois += [key[-1], -1]
            else:
                if (set_fraction < key[1]):
                    t_antes += [key[-1], -1]
                    t_depois += [key[1], 1]
                else:
                    if (set_fraction == key[-1]):
                        t_antes += [key[-1], -1]
                        t_depois += [key[0], 0]
                    else:
                        t_antes += [key[-2], -2]
                        t_depois += [key[-1], -1]
                    

        else:
            # print(f"set_fraction = {set_fraction}")
            for t in range(1, len(key)):
                # print(f"set_fraction : {set_fraction} ; key[{t}] : {key[t]}")
                if set_fraction < key[t]:
                    t_antes += [key[t-1], t-1]
                    t_depois += [key[t], t]
                    # print(f"t_antes : {t_antes} ; t_depois : {t_depois}")
                    break
        if closed:
            s = (set_fraction - t_antes[0])/(t_depois[0] - t_antes[0])
            
            T1 = (new_keyValue[t_depois[1]] - new_keyValue[t_antes[1]-1])/2
            T2 = (new_keyValue[-(len(key) - (t_depois[1]+1))] - new_keyValue[t_antes[1]])/2
            
            # print(f"len(key) = {len(key)} => (new_keyValue[{t_depois[1]}] - new_keyValue[{t_antes[1]-1}])/2\nnew_keyValue[{-(len(key) - (t_depois[1]))}] - new_keyValue[{t_antes[1]+1}] / 2")
        else:
            T1, T2 = np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])
        
        S = np.array([[s**3],
                      [s**2],
                      [s],
                      [1]])
        H = np.array([[2,-2,1,1], [-3,3,-2,-1], [0, 0, 1, 0], [1, 0, 0, 0]])
        
        # if(closed):
        #     C = np.array([new_keyValue[t_antes[1]], new_keyValue[t_depois[1]], T0, T0])
        # else:
        # print(f"T1 = {T1} ; T2 = {T2}")
        # print(f"C = {np.array([new_keyValue[t_antes[1]], new_keyValue[t_depois[1]]])}")
        C = np.array([new_keyValue[t_antes[1]], new_keyValue[t_depois[1]], T1, T2])
            
        # print(f"S = {S}")
        # print(f"H = {H}")
        # print(f"C = {C}")

        V_s = np.dot(np.dot(np.matrix.transpose(S), H), C)
        # print(f"V_s = {np.ndarray.tolist(V_s[0])} -- {type(np.ndarray.tolist(V_s[0]))}")
        
        return np.ndarray.tolist(V_s[0])

    @staticmethod
    def orientationInterpolator(set_fraction, key, keyValue):
        """Interpola entre uma lista de valores de rotação especificos."""
        # Interpola rotações são absolutas no espaço do objeto e, portanto, não são cumulativas.
        # Uma orientação representa a posição final de um objeto após a aplicação de uma rotação.
        # Um OrientationInterpolator interpola entre duas orientações calculando o caminho mais
        # curto na esfera unitária entre as duas orientações. A interpolação é linear em
        # comprimento de arco ao longo deste caminho. Os resultados são indefinidos se as duas
        # orientações forem diagonalmente opostas. O campo keyValue possui uma lista com os
        # valores a serem interpolados, key possui uma lista respectiva de chaves
        # dos valores em keyValue, a fração a ser interpolada vem de set_fraction que varia de
        # zeroa a um. O campo keyValue deve conter exatamente tantas rotações 3D quanto os
        # quadros-chave no key.

        # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
        print("OrientationInterpolator : set_fraction = {0}".format(set_fraction))
        print("OrientationInterpolator : key = {0}".format(key)) # imprime no terminal
        print("OrientationInterpolator : keyValue = {0}".format(keyValue))

        # Abaixo está só um exemplo de como os dados podem ser calculados e transferidos
        value_changed = [0, 0, 1, 0]

        return value_changed

    # Para o futuro (Não para versão atual do projeto.)
    def vertex_shader(self, shader):
        """Para no futuro implementar um vertex shader."""

    def fragment_shader(self, shader):
        """Para no futuro implementar um fragment shader."""
