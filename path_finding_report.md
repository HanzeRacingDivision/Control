# Driverless Code Design Report – Path Finding

## INTRODUCTION
[//]: # (Write a short introduction about the component that you have been working on. Also give a short explanation of the chapters to come.)
The path_finding function takes a list of triangles between blue and yellow cones, generated by the Delaunay
triangularization function.
This is converted to an array of edges between cones of opposite color.
Then it first finds the order in which they are position on the track.
Secondly, it will determine the midpoints between the edges and return them as an ordered list, with the first element
being the first upcoming midpoint.

## RULES AND DEMANDS
[//]: # (Are there any rules to which your part has to comply? Are there any other demands from other teams or people to which your part has to comply?)
There are no specific rules to constrain this element of the control pipeline.
The function does however need to comply with the input it is given by the Delaunay triangularization and the output
that is expected by the generate_point_along_path method.
Specifically, the input consists of a numpy array of triangles, where each node is a natural number that refers to an
index in the second argument.
This second argument is an ordered list of dictionaries, each describing a cone.
The output is in the form a numpy array of tuples, representing coordinates.
These coordinates represent x-y-coordinates and are ordered along the path we want to take.

## RESEARCH
[//]: # (What is the research you have done? Link to any important documents. Summarize why you made the decision you have made.) 
I did not do research for this part, as my solution arises naturally from the given problem.

## CODING PROCESS
[//]: # (Summarize how the coding process went.)
The coding process was not special.
I first coded something that would work for a simplified input and then adapted it to the actual input I would get from
the Delaunay triangularization.

## TESTING
[//]: # (Explanation and results of unit testing, chain testing, and practical tests.) 
There has only been unit testing and partial chain testing so far, which both was successful.
In the chain testing combined with Delaunay, I figured out that certain points would appear multiple times and was able
to solve that problem by only generating unique edges from the input.

## CONCLUSION/DISCUSSION
The program is quite simple and not very fast, but it should work and be fast enough because of the small size of any
input it would get, which should be a few dozen cones and even fewer triangles at most.
