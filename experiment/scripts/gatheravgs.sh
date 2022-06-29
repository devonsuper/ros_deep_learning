
outputdir=$1

echo "" > $outputdir/averages.txt

for file in $outputdir/*
do
    echo $file:     $(tail -3 $file | grep "average") >> $outputdir/averages.txt

done