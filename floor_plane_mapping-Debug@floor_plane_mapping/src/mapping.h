#ifndef MAPPING_H_INCLUDED
#define MAPPING_H_INCLUDED

#define nullptr 0;
// Flags for the data contained in the matrix
// Unknown & Traversable = Traversable
// All the rest & NonTraversable = NonTraversable
enum CellState
{
	NonTraversable = 0,
	Traversable = 1,
	Unknown = 3	// Default state
};

class Cell{
public:
	double x;
	double y;
	CellState cellState;
};

#endif

