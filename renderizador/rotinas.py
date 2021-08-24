#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

"""
Rotinas de operação de nós X3D.

Desenvolvido por:
Disciplina: Computação Gráfica
Data:
"""

import gpu          # Simula os recursos de uma GPU

#################################################################################
# NÃO USAR MAIS ESSE ARQUIVO. AS ROTINAS DEVEM SER IMPLEMENTADAS AGORA NO gl.GL #
#################################################################################

# web3d.org/documents/specifications/19775-1/V3.0/Part01/components/geometry2D.html#Polypoint2D
def polypoint2D(point, colors):
    """Função usada para renderizar Polypoint2D."""
    # Nessa função você receberá pontos no parâmetro point, esses pontos são uma lista
    # de pontos x, y sempre na ordem. Assim point[0] é o valor da coordenada x do
    # primeiro ponto, point[1] o valor y do primeiro ponto. Já point[2] é a
    # coordenada x do segundo ponto e assim por diante. Assuma a quantidade de pontos
    # pelo tamanho da lista e assuma que sempre vira uma quantidade par de valores.
    # O parâmetro colors é um dicionário com os tipos cores possíveis, para o Polypoint2D
    # você pode assumir o desenho dos pontos com a cor emissiva (emissiveColor).

    # O print abaixo é só para vocês verificarem o funcionamento, DEVE SER REMOVIDO.
    print("Polypoint2D : pontos = {0}".format(point)) # imprime no terminal pontos
    print("Polypoint2D : colors = {0}".format(colors)) # imprime no terminal as cores
    # Exemplo:
    gpu.GPU.set_pixel(3, 1, 255, 0, 0) # altera um pixel da imagem (u, v, r, g, b)
    # cuidado com as cores, o X3D especifica de (0,1) e o Framebuffer de (0,255)
   
    i = 0
    while (i < len(point)-1):
        gpu.GPU.set_pixel(int(point[i]), int(point[i+1]), 255, 0, 255)
        i=i + 2

# web3d.org/documents/specifications/19775-1/V3.0/Part01/components/geometry2D.html#Polyline2D
def polyline2D(lineSegments, colors):
    """Função usada para renderizar Polyline2D."""
    # Nessa função você receberá os pontos de uma linha no parâmetro lineSegments, esses
    # pontos são uma lista de pontos x, y sempre na ordem. Assim point[0] é o valor da
    # coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto. Já point[2] é
    # a coordenada x do segundo ponto e assim por diante. Assuma a quantidade de pontos
    # pelo tamanho da lista. A quantidade mínima de pontos são 2 (4 valores), porém a
    # função pode receber mais pontos para desenhar vários segmentos. Assuma que sempre
    # vira uma quantidade par de valores.
    # O parâmetro colors é um dicionário com os tipos cores possíveis, para o Polyline2D
    # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).

    print("Polyline2D : lineSegments = {0}".format(lineSegments)) # imprime no terminal
    print("Polyline2D : colors = {0}".format(colors)) # imprime no terminal as cores
    # Exemplo:
    # codigo baseado na roseta stone http://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#Python
    pos_x = gpu.GPU.width//2
    pos_y = gpu.GPU.height//2
    # gpu.GPU.set_pixel(pos_x, pos_y, 255, 0, 0) # altera um pixel da imagem (u, v, r, g, b)

    x0, y0, x1, y1 = lineSegments[0], lineSegments[1], lineSegments[2], lineSegments[3]
    
    dx = round(abs(x1 - x0))
    dy = round(abs(y1 - y0))
    
    # m = dy/dx
    sx = -1 if x0 > x1 else 1 # sx = 1
    sy = -1 if y0 > y1 else 1 # sy = 1

    x, y = x0, y0
    r, g, b = int(colors["emissiveColor"][0]*255), int(colors["emissiveColor"][1]*255), int(colors["emissiveColor"][2]*255)

    if (dx > dy):
        err = dx // 2
        while (round(x) != round(x1)):
            gpu.GPU.set_pixel(int(x), int(y), r, g, b)
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy // 2
        while(round(y) != round(y1)):
            gpu.GPU.set_pixel(int(x), int(y), r, g, b)
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    # gpu.GPU.set_pixel(int(x), int(y), r, g, b)
    # (2, 4) -> (6, 8)
    # x0, y0, x1, y1 = lineSegments[0], lineSegments[1], lineSegments[2], lineSegments[3] # x0 = 2, y0 = 4, x1 = 6, y1 = 8
    # dx = int(abs(x1 - x0)) # dx = 4
    # dy = int(abs(y1 - y0)) # dy = 4
    # x, y = int(x0), int(y0) # x = 2, y = 4
    # sx = -1 if x0 > x1 else 1 # sx = 1
    # sy = -1 if y0 > y1 else 1 # sy = 1
    # if dx > dy: # false -> vai para o else
    #     err = dx / 2.0
    #     while x != x1:
    #         gpu.GPU.set_pixel(int(x), int(y), 255, 0, 0)
    #         err -= dy
    #         if err < 0:
    #             y += sy
    #             err += dx
    #         x += sx
    # else: # <-
    #     err = dy / 2.0 # err = 2.0
    #     while y != y1: # while 7 != 8 
    #         gpu.GPU.set_pixel(int(x), int(y), 255, 0, 0) # pinta pixel
    #         err -= dx # err = 2.0
    #         if err < 0:
    #             x += sx # x = 4
    #             err += dy # err = 2.0
    #         y += sy # y = 8

    # gpu.GPU.set_pixel(int(x), int(y), 255, 0, 0) # pinta o pixel (4, 8)


# web3d.org/documents/specifications/19775-1/V3.0/Part01/components/geometry2D.html#TriangleSet2D
def triangleSet2D(vertices, colors):
    # TODO
    """Função usada para renderizar TriangleSet2D."""
    # Nessa função você receberá os vertices de um triângulo no parâmetro vertices,
    # esses pontos são uma lista de pontos x, y sempre na ordem. Assim point[0] é o
    # valor da coordenada x do primeiro ponto, point[1] o valor y do primeiro ponto.
    # Já point[2] é a coordenada x do segundo ponto e assim por diante. Assuma que a
    # quantidade de pontos é sempre multiplo de 3, ou seja, 6 valores ou 12 valores, etc.
    # O parâmetro colors é um dicionário com os tipos cores possíveis, para o TriangleSet2D
    # você pode assumir o desenho das linhas com a cor emissiva (emissiveColor).
    print("TriangleSet2D : vertices = {0}".format(vertices)) # imprime no terminal
    print("TriangleSet2D : colors = {0}".format(colors)) # imprime no terminal as cores
    # Exemplo:
    gpu.GPU.set_pixel(24, 8, 255, 255, 0) # altera um pixel da imagem (u, v, r, g, b)
