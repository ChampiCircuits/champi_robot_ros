https://www.youtube.com/watch?v=4PUiDmD5dkg

### Compilation : 

cd
sudo apt-get install libzmq3-dev libboost-dev
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
mkdir build ; cd build
cmake ..
make
sudo make install


### Utilisation
Dans CMakeLists.txt j'ai fait un includedirectory qui pointe vers ~/
Comme c'est là qu'on a cloné et installé BT.CPP
Ptet trouver un meilleur moyen ?

En tout cas faudrait automatiser le process et mettre des checks

### Liens
https://github.com/BehaviorTree/BehaviorTree.CPP
https://github.com/BehaviorTree/btcpp_sample/blob/main/main.cpp

### Comment je vois le truc :

- on démarre le match via l'UI
- les params choisis dans l'UI sont envoyés au BT_Manager, un script cpp qui lance les BT
- le BT_Manager décide par exemple de lancer le BT "blue_homologation"

- le BT tourne, et la première action est par ex se déplacer au point A
- 