float sigmoid(float x)
{
     float exp_value;
     float return_value;

     exp_value = exp((double) -x);
     return_value = 1 / (1 + exp_value);

     return return_value;
}

void print_vector(std::vector<float> v) {
    for(int i=0; i<v.size(); ++i) {
        std::cout<<v[i]<<" ";
    }
    std::cout<<std::endl;
}