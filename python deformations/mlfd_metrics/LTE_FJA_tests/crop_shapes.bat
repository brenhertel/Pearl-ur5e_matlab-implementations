@echo off
setlocal enabledelayedexpansion

set "lasa_names=Angle BendedLine CShape DoubleBendedLine GShape heee JShape JShape_2 Khamesh Leaf_1 Leaf_2 Line LShape NShape PShape RShape Saeghe Sharpc Sine Snake Spoon Sshape Trapezoid Worm WShape Zshape Multi_Models_1 Multi_Models_2 Multi_Models_3 Multi_Models_4"

set metric_names=Haussdorff Frechet Curvature_Conservation2 Endpoint_Convergence Curve_Length PCM Area DTW Curve_Length2

for %%b in (%lasa_names%) do (
   echo %%b
   set "name=sum_of_dists"
   echo !name!
   set "fpath=!name!_%%b"
   echo !fpath!
   cd !fpath!
   set "fname=%%b_Original.png"
   set "fname_out=%%b_shaved.png"
   magick convert !fname! -shave 80x60 !fname_out!
   echo/
   cd ..
)