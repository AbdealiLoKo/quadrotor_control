#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h> 
#include <time.h>
#include "utils.cpp"

class Neuron {
public:
    int n_inputs;
    std::vector<float> weights;

    Neuron(int n);
    float random_weight(float min_w=-0.1, float max_w=0.1);
    float get_output(std::vector<float> inp); 
    float activate(float inp);
};

class NeuralLayer {
public:
    int n_neurons;
    std::vector<Neuron> neurons;

    NeuralLayer(int n, int inp); 
    std::vector<float> get_outputs(std::vector<float> inp);
};

class NeuralNetwork {
    int n_inputs;
    int n_hidden_layers;
    int n_outputs;
    int n_per_hidden_layer;
    std::vector<NeuralLayer> layers;
    std::vector<float> weights;

public:    
    NeuralNetwork(int n_i=0, int n_o=0, int n_h=0, int n_per_h=0);
     int get_n_weights();
     void get_weights(std::vector<float> &weights);
     void set_weights(std::vector<float> w);
     void get_value(std::vector<float> &input, std::vector<float> &output);
};
