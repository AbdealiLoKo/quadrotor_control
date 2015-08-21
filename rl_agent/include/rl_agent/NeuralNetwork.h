#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h> 
#include <time.h>

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

class Neuron {
public:
    int n_inputs;
    std::vector<float> weights;

    Neuron(int n) {
        n_inputs = n;
        std::vector<float> v;
        for(int i=0; i<n+1; ++i) {
            v.push_back(random_weight());         //Insert random number
        }
        weights = v;
    }

    ~Neuron() {
    }

    float random_weight(float min_w=-0.1, float max_w=0.1) {
        float w = min_w + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_w-min_w)));
        return w;
    }

    float get_output(std::vector<float> inp) {
        float output = 0;
        for(int i=0; i<inp.size(); ++i) {
            output += inp[i] * weights[i];
        }
        output += weights[n_inputs];
        return activate(output);
    }

    float activate(float inp) {
        return sigmoid(inp);
    }
};

class NeuralLayer {
public:
    int n_neurons;
    std::vector<Neuron> neurons;

    NeuralLayer(int n, int inp) {
        n_neurons = n;
        std::vector<Neuron> v;
        for(int i=0; i<n_neurons; ++i) {
            v.push_back(Neuron(inp));
        }
        neurons = v;
    }

    ~NeuralLayer() {
    }

    std::vector<float> get_outputs(std::vector<float> inp) {
        std::vector<float> v(n_neurons); 
        for(int i=0; i<n_neurons; ++i) {
           v[i] = neurons[i].get_output(inp); 
        }
        return v;
    }
};

class NeuralNetwork {
    int n_inputs;
    int n_hidden_layers;
    int n_outputs;
    int n_per_hidden_layer;
    std::vector<NeuralLayer> layers;
    std::vector<float> weights;

public:    
    NeuralNetwork(int n_i=0, int n_o=0, int n_h=0, int n_per_h=0) {
        n_inputs = n_i;
        n_outputs = n_o;
        n_per_hidden_layer = n_per_h;
        n_hidden_layers = n_h;

        // Initialize random seed generator
        
        srand(time(NULL));
      
        // Create the input layer
        NeuralLayer input_layer = NeuralLayer(n_inputs, 0);
        layers.push_back(input_layer);

        // Number of inputs for next layer
        int next_inp = n_inputs;
        for(int i=0; i<n_hidden_layers; ++i) {
            layers.push_back(NeuralLayer(n_per_hidden_layer, next_inp));
            next_inp = n_per_hidden_layer;
        }

        NeuralLayer output_layer = NeuralLayer(n_outputs, next_inp);
        layers.push_back(output_layer);

        std::vector<float> temp(get_n_weights());
        get_weights(temp);
        weights = temp;
    }

    ~NeuralNetwork() {

    }

    int get_n_weights() {
        return ((n_inputs+1)*n_per_hidden_layer + n_outputs*(n_per_hidden_layer+1) + (n_hidden_layers-1)*((n_per_hidden_layer+1)*n_per_hidden_layer));
    }

    void get_weights(std::vector<float> &weights) {
        // Faster method would be to keep a vector of weights here
        // Unless the network is very big, speed shouldnt be an issue
        int no=0;
        for(std::vector<NeuralLayer>::iterator it=layers.begin(); it != layers.end(); ++it) {
            // Skip the input layer
            if(it != layers.begin()) {
                std::vector<Neuron> neurons = it->neurons;
                for(std::vector<Neuron>::iterator itn=neurons.begin(); itn != neurons.end(); ++itn) {
                    Neuron n = *itn;                  
                    for(int i=0; i<n.n_inputs+1; i++) {
                        weights[no++] = n.weights[i];
                    }
                }
            }
        }
    }

    void set_weights(std::vector<float> w) {
        weights = w;
        for(std::vector<NeuralLayer>::iterator it=layers.begin(); it != layers.end(); ++it) {
            // Skip the input layer
            int no=0;
            if(it != layers.begin()) {
                std::vector<Neuron> neurons = it->neurons;
                for(std::vector<Neuron>::iterator itn=neurons.begin(); itn != neurons.end(); ++itn) {
                    Neuron n = *itn;
                    for(int i=0; i<n.n_inputs+1; i++) {
                        n.weights[i] = weights[no++];
                    }
                }
            }
        }
    }

    void value(std::vector<float> &input, std::vector<float> &output) {
        for(std::vector<NeuralLayer>::iterator it=layers.begin(); it != layers.end(); ++it) {
            std::cout<<"Input at layer ";
            print_vector(input);

            output.clear();
            output = it->get_outputs(input);

            // Next input comes from output of this layer
            input = output;

            std::cout<<"Output at layer ";
            print_vector(output);
        }
    }       
};