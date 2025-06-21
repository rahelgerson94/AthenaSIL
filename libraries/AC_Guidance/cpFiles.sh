#!/bin/bash

refDir="/Users/rahelmizrahi/Documents/PlatformIO/Projects/AthenaSim/AthenaSimCpp"

rsync -av "$refDir/Guidance/"*.cpp .
rsync -av "$refDir/Guidance/"*.h .

rsync -av "$refDir/Common/"*.cpp .
rsync -av "$refDir/Common/"*.h .
