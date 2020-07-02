@echo off
setlocal enabledelayedexpansion

for %%a in (SVM*) do (
	set "fname=%%a"
	echo !fname!
	set "fname_out=cropped_imgs/%%a_shaved.png"
	magick convert "!fname!" -crop 496x365+80+60 "!fname_out!"
	echo/
)