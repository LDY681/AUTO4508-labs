# Makefile for quadtree and astar

# Default rule
all: quadtree.x ../lab4/astar.x

quadtree.x:
	gccsim -o quadtree.x quadtree.c image.c

../lab4/astar.x:
	gccsim -o ../lab4/astar.x ../lab4/astar.c

clean:
	$(RM) *.x ../lab4/*.x
