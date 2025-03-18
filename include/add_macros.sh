for i in $(grep -r -l -E -e "Eigen::(Vector|Matrix)[a-zA-Z0-9]* [a-zA-Z_0-9]+;" | xargs grep -L -E -e "EIGEN_MAKE_ALIGNED_OPERATOR_NEW")
do
    nvim ${i}      
done
