#!/bin/bash

refDir=$1

rsync -av "$refDir/Guidance/"*.cpp .
rsync -av "$refDir/Guidance/"*.h .

rsync -av "$refDir/Common/"*.cpp .
rsync -av "$refDir/Common/"*.h .
