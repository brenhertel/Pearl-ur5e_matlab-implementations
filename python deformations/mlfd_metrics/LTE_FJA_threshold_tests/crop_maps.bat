@echo off
setlocal enabledelayedexpansion

set "lasa_names=Angle BendedLine CShape DoubleBendedLine GShape heee JShape JShape_2 Khamesh Leaf_1 Leaf_2 Line LShape NShape PShape RShape Saeghe Sharpc Sine Snake Spoon Sshape Trapezoid Worm WShape Zshape Multi_Models_1 Multi_Models_2 Multi_Models_3 Multi_Models_4"

set "metric_names=SSE SEA"

for %%a in (%lasa_names%) do (
	rem echo %%b
	for %%b in (%metric_names%) do (
		set "fpath=%%b_%%a"
		echo !fpath!
		cd !fpath!
		set "fname=%%bComparison.png"
		set "fname_out=%%bComaprison_shaved.png"
		magick convert !fname! -shave 125x45 !fname_out!
		echo/
		cd ..
	)
)