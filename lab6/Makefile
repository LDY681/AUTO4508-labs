# Makefile for brushfire and astar

# Default rule
all: brushfire.x ../lab4/astar.x

brushfire.x:
	gccsim -o brushfire.x brushfire.c image.c

../lab4/astar.x:
	gccsim -o ../lab4/astar.x ../lab4/astar.c

clean:
	$(RM) *.x ../lab4/*.x
