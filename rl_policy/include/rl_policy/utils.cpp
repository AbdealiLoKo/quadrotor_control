float sigmoid(float x)
{
     float exp_value;
     float return_value;

     exp_value = exp((double) -x);
     return_value = 1 / (1 + exp_value);

     return return_value;
}
