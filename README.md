Installation
============

```
git clone <...>
cd cpp-utils
mkdir -p build/Release # use "Debug" if you want to build in debug
cd build/Release
cmake ../..
make
sudo make install
sudo ldconfig
```

Uninstall
=========

```
cd build/Release
sudo make uninstall
```

Information
===========

Use 

```
./pathfinding-map-perturbator --help
```

to look for all the options.
As an example of usage:

```
./pathfinding-map-perturbator --map-filename="square03.map" --perturbation-kind="RANDOM" --random-seed=0 --output-main-directory="." --query-per-perturbated-map=10 --number-of-perturbation-pystring="20" --perturbation-range-pystring="[0.5, 1.5]" --landmark-path-pystring="landmark-database-{MAPNAME}-{PERTURBATEDMAPID}.landmarkdb" --output-temp-directory="." --output-main-csv-pystring="generated-{MAPNAME}-{PERTURBATEDMAPID}.csv" --output-json-map-pystring="generated-{MAPNAME}-{PERTURBATEDMAPID}.json" --output-json-original-map-pystring="original-{MAPNAME}.json" --number-of-perturbated-map-pystring="3"
```

the following will create 3 perturbated maps, each with 20 perturbated edges (each perturbation affect the edge base cost with a multiplier between [0.5, 1.5]), and we will perform 10 queries on each perturbated map.