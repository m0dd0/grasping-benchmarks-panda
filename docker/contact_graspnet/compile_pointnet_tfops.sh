CUDA_INCLUDE=' -I/usr/local/cuda/include/'
CUDA_LIB=' -L/usr/local/cuda/lib64/'
TF_CFLAGS=$(python3 -c 'import tensorflow as tf; print(" ".join(tf.sysconfig.get_compile_flags()))')
TF_LFLAGS=$(python3 -c 'import tensorflow as tf; print(" ".join(tf.sysconfig.get_link_flags()))')
cd /home/ContactGraspnetBenchmark/contact_graspnet/orig/pointnet2/tf_ops/sampling

nvcc -c -o tf_sampling_g.cu.o tf_sampling_g.cu ${CUDA_INCLUDE} ${TF_CFLAGS} -D GOOGLE_CUDA=1 -x cu -Xcompiler -fPIC

g++ -std=c++11 -shared -o tf_sampling_so.so tf_sampling.cpp 
# tf_sampling_g.cu.o ${CUDA_INCLUDE} ${TF_CFLAGS} -fPIC -lcudart ${TF_LFLAGS} ${CUDA_LIB}

g++ -std=c++11 tf_sampling.cpp tf_sampling_g.cu.o -o tf_sampling_so.so -shared -fPIC -I /usr/local/lib/python2.7/dist-packages/tensorflow/include -I /usr/local/cuda-8.0/include -I /usr/local/lib/python2.7/dist-packages/tensorflow/include/external/nsync/public -lcudart -L /usr/local/cuda-8.0/lib64/ -L/usr/local/lib/python2.7/dist-packages/tensorflow -ltensorflow_framework -O2 -D_GLIBCXX_USE_CXX11_ABI=0

# echo 'testing sampling'
# python3 tf_sampling.py
 
# cd ../grouping

# nvcc -std=c++11 -c -o tf_grouping_g.cu.o tf_grouping_g.cu \
#  ${CUDA_INCLUDE} ${TF_CFLAGS} -D GOOGLE_CUDA=1 -x cu -Xcompiler -fPIC

# g++ -std=c++11 -shared -o tf_grouping_so.so tf_grouping.cpp \
#  tf_grouping_g.cu.o ${CUDA_INCLUDE} ${TF_CFLAGS} -fPIC -lcudart ${TF_LFLAGS} ${CUDA_LIB}

# echo 'testing grouping'
# python3 tf_grouping_op_test.py

 
# cd ../3d_interpolation
# g++ -std=c++11 tf_interpolate.cpp -o tf_interpolate_so.so -shared -fPIC -shared -fPIC ${TF_CFLAGS} ${TF_LFLAGS} -O2
# echo 'testing interpolate'
# python3 tf_interpolate_op_test.py

