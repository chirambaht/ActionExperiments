arr=( 0 1 2 3 4 5 9 19 39 199)

for i in "${arr[@]}"
do
	./collect_data "$i"
done
