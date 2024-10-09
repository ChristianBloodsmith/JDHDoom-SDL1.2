# Wolfenstein/DOOM style software renderers

![project screenshot](screen/image.png)

Original files - - -
* `src/main_doom.c`: a DOOM-style software renderer
* `src/main_wolf.c`: a Wolfenstein 3D-style software renderer

My additions - - -
* `src/doomsdl.c`: conversion of the DOOM-style software renderer to use SDL 1.2 instead. May add texture support later.

### Building & Running

`$ make doom|wolf|all`, binaries are `bin/doom` and `bin/wolf` respectively
