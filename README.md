# DIY Surface Scanner

[![License: MIT](https://shields.io)](https://opensource.org)

## Abstract

Advances in technology in the virtual three-dimensional space have provoked the need for affordable options
for the broader market. This paper demonstrates that an affordable 3D surface scanning setup can be created
for less than $100 that is capable of sub-millimeter resolution. This is achieved through laser triangulation,
the use of open source python libraries such as OpenCV to capture and process the images and the Poisson
Surface Reconstruction algorithm to convert the scanned 3D point cloud into a watertight mesh. While
analysis of the resulting mesh after scanning a sample surface indicated smoothing artifacts due to gaps in
the points cloud via laser clipping, the artifacts were less than half a millimeter in size and did not affect the
resultant mesh in any significant manner.

See rest of paper and sample cover letter under the documents folder.

## Features
- Sub-millimeter accuracy
- Auto zeroing
- Friendly commandline interface
- Affordable (< $100 for parts)
- BPA, BPA with MLS and PSR surface reconstruction algorithm options

## Built With
- Python
- C++
- OpenCV
- Open3D
