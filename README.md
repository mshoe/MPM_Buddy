# Material Point Method C++ Tool

This program was made using C++ and OpenGL compute shaders.
It also uses OpenGL for rendering and ImGui for GUI.
I am using this program for my own research in physics-based animation.
I hope other users can also use this for their MPM needs or to just play around. Right now they would need to build it themselves, but I plan to release this as a standalone open-source software once I am happy with what I've done with it.

__Current Research (at UofT DGP):__<br/>
MPM control algorithms.
Natural triggers for landslides in MPM.

__Read the Wiki for more information:__<br/>
https://github.com/mshoe/MPM_Geometry/wiki<br/>

__Video Gallery:__<br/>
https://www.youtube.com/channel/UC4qfNTQgecwtluc0M5HRnTw<br/>

__Nice GUI:__<br/>
![pic](gifs/Random/niceGUI.PNG)<br/>

__Gif Gallery:__<br/>
Rendering speed colored points vs marching squares for green fluid:<br/>
![gif](gifs/AmorphousObjectsAlive/marchingSquaresVsPoints.gif)<br/>

Rendering points with elastic potential energy:<br/>
![gif](gifs/RenderingPointsWithEnergy.gif)<br/>

Different grid sizes:<br/>
![gif](gifs/smallerGrid.gif)<br/>

Elastic deformation gradient modification (example 1) (colors are stress visualization):<br/>
![gif](gifs/AmorphousObjectsAlive/lineBigger.gif)<br/>

Elastic deformation gradient modification (example 2) (colors are speed visualization):<br/>
![gif](gifs/Random/dgpDefGrad.gif)<br/>

Mathematical trigger for landslide (colors are speed visualization):<br/>
![gif](gifs/polygonSlopeSSR.gif)<br/>


Material Point Method (MPM) is an Eulerian-Lagrangian hybrid algorithm for simulating continuum materials.

__Build Instructions for Windows__<br/>
Clone this repo, open MPM/MPM.vcxproj in Visual Studio 2019, and build. This program is using NVidia OpenGL extensions: GL_NV_shader_atomic_float64. A sufficient NVidia GPU that supports OpenGL 4.5 is required for the program to run properly (https://www.khronos.org/registry/OpenGL/extensions/NV/NV_shader_atomic_float64.txt). Also, make sure your graphics drivers are up to date.