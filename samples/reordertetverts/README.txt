Command line program that processes .node/.ele files to reorder the tetrahedral vertices, switching between 
TetGen and Stellar conventions.  Also strips out any unreferenced vertices.

FEMFX samples and library use the same convention as TetGen, which is that verts 012 are CCW as seen from 3, 
vs. the Stellar convention that verts 123 are CCW as seen from 0.
