A = importdata('DME.txt',' ');
tri = delaunay(A(:,1),A(:,2));
h = trisurf(tri,A(:,1),A(:,2),A(:,3),A(:,4));
