#!/bin/bash
rm -rf ../launch
mkdir ../launch
echo "<launch>" > ../launch/t5k.launch
for OUT in $(ls src/ -1 | sed -e 's/\..*$//')
do
	echo "<node name=\"$OUT\" pkg=\"turtle5k\" type=\"$OUT\" />" >> ../launch/t5k.launch
done
echo "</launch>" >> ../launch/t5k.launch
