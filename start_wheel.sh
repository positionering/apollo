docker exec realsense_dev_twizy1 ./realsenset265/build/pose-grej &
p1pid=$!
read -n 1 -s -r -p "enter för att stänga ner INTE CTRL-C"
kill $p1pid
