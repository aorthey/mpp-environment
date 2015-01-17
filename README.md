mpp-environment
=======
mpp-environment belongs to a set of packages to conduct motion planning for a sliding humanoid robot in arbitrary environments via motion prior informations.
### Description of package
Package precomputes structural information about the free workspace of a given URDF file
### Input: 
URDF file specification of an environment
### Output:
 * Walkable Surfaces S_1,...,S_K
 * Surface Stacks E_1,...,E_K
 * Interconnection Stacks I_1,...,I_{K-1}
 * Connectivity Graph G_S

### Dependencies
 * BeautifulSoup -- parsing of environment URDF
 * numpy -- mathematical computations
 * scipy -- convexhull algorithm
 * networkx -- graph structure for surface connectivity
 * pickle -- store and load of python structures

