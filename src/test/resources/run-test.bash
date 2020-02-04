#!/bin/bash

./pathfinding-map-perturbator \
	--map-filename="square03.map" \
	--random-seed=0 \
	--output-main-directory="." \
	--output-temp-directory="." \
	--query-per-perturbated-map=10 \
	--number-of-perturbated-map-pystring="3" \
	--number-of-perturbation-pystring="20" \
	--perturbation-kind="RANDOM" \
	--perturbation-range-pystring="[0.5, 1.5]" \
	--landmark-path-pystring="landmark-database-{MAPNAME}-{PERTURBATEDMAPID}.landmarkdb" \
	\
	--output-main-perturbated-map-image-pystring="perturbated-map-{MAPNAME}-{PERTURBATEDMAPID}.jpeg" \
	--output-json-original-map-pystring="original-{MAPNAME}.json" \
	--output-json-map-pystring="generated-{MAPNAME}-{PERTURBATEDMAPID}.json" \
	--output-main-solution-image-pystring="generated-solutions-{MAPNAME}-{PERTURBATEDMAPID}-{QUERYID}.jpeg" \
	\
	--output-main-csv-pystring="generated-{MAPNAME}-{PERTURBATEDMAPID}.csv"
