#include <rl_policy/NeuralNetwork.h>

Neuron::Neuron(int n) {
    n_inputs = n;
    weights.resize(n);
    for(int i=0; i<n+1; ++i) {
    	float k = random_weight();
        weights[i] = k;         //Insert random number
    }
}

float Neuron::random_weight(float min_w, float max_w) {
    float w = min_w + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_w-min_w)));
    return w;
}

float Neuron::get_output(std::vector<float> inp) {
    float output = 0;
    for(int i=0; i<inp.size(); ++i) {
        output += inp[i] * weights[i];
    }
    output += weights[n_inputs];
    return activate(output);
}

float Neuron::activate(float inp) {
    return sigmoid(inp);
}

NeuralLayer::NeuralLayer(int n, int inp) {
    n_neurons = n;
    std::vector<Neuron> v;
    for(int i=0; i<n_neurons; ++i) {
        v.push_back(Neuron(inp));
    }
    neurons = v;
}

std::vector<float> NeuralLayer::get_outputs(std::vector<float> inp) {
    std::vector<float> v(n_neurons); 
    for(int i=0; i<n_neurons; ++i) {
       v[i] = neurons[i].get_output(inp); 
    }
    return v;
}

NeuralNetwork::NeuralNetwork(int n_i, int n_o, int n_h, int n_per_h) {
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

    weights.resize(get_n_weights());
    get_weights(weights);
}

int NeuralNetwork::get_n_weights() {
    return ((n_inputs+1)*n_per_hidden_layer + n_outputs*(n_per_hidden_layer+1) + (n_hidden_layers-1)*((n_per_hidden_layer+1)*n_per_hidden_layer));
}

void  NeuralNetwork::get_weights(std::vector<float> &weights) {
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

void  NeuralNetwork::set_weights(std::vector<float> w) {
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

void  NeuralNetwork::get_value(std::vector<float> &input, std::vector<float> &output) {
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