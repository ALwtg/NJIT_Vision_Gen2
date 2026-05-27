sleep 5
cd ~/Desktop/NJIT_Vision/NJIT_Vision_Gen2/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./infantry.sh"