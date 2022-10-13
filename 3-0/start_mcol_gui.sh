#!/bin/bash

#From config/MASTER_RELEASE extract paths to modules that provide qegui screens
export amc100path=`grep "AMC100" config/AMC100RELEASE | cut -d'=' -f2`

#From module top, add offset path to qegui screens
export amc100path=$amc100path/AMC100Sup/op/ui

echo $amc100path

#QEGUI path to screens
export QE_UI_PATH=$amc100path

#Determine Qt version
export QTVERSION=`qmake --version | grep Qt | cut -b 18`

#Determine Qt style to use from version
if [ ${QTVERSION} = "4" ]; then
#Qt4 detected
   export QTSTYLE="plastique"
elif [ ${QTVERSION} = "5" ]; then
#Qt5 detected
   export QTSTYLE="fusion"
else
#Unknown assume Qt4
   export QTSTYLE="plastique"
fi

#Invoke QEGUI
qegui -style ${QTSTYLE} -e -m "P=SR03ID01HU03COL01:, M1=X, M2=Y" MicroCollimator.ui &

