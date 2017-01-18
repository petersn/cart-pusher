#! /usr/bin/python

import string, os
import pygame
pygame.init()

font = pygame.font.SysFont("Courier", 100)

for char in string.printable:
	if char != " " and char in string.whitespace:
		continue
	index = ord(char)
	img = font.render(char, True, (255, 255, 255))#.convert(24)
	pygame.image.save(img, "%i.png" % index)

