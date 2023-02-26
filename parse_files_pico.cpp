/*
 * Author: Sadie Lauser
 * Date: 2/5/23
 * Description: parses data and key files, verifies proper formatting, then separates data into two files based on keys
 * Pre-conditions: data file has x sections of SAMPLES_SIZE (const defined below) lines, each that have 2 numbers (rms acceleration, piezo reading)
 * Post-conditions:
 * Post-conditions: the formatting (above) is verified in both files and if correct, it appends each sample's data to either the net_shot.csv or the rim_shot.csv 
 * 					file depending on which key ('r' or 'n'/'p') is specified for that sample. If the 'x' key is specified, that sample is not written to either file
 * 					(it is discarded). If one of the files does not have proper formatting the program ends before writing to any files.
*/

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <vector>

using namespace std;

#define SAMPLE_SIZE 350    //may change this 
#define RMS_IDX     0
#define PIEZO_IDX   1

bool open_file(ifstream&, string);
int verify_keys_file(ifstream&);
int verify_data_file(ifstream&);
void write_to_file(ofstream&, vector< vector<float> >&);
void get_keys(ifstream&, vector<char>&);
void print_keys(vector<char>);
int get_sample(ifstream&, vector< vector<float> >&);


int main(int argc, char** argv){
	ifstream fin_data;
	ifstream fin_keys;
	string data_name;
	string keys_name;
    string line;	
    int num_files = 1;

	//get file names from command line or user input
	if(argc == 3){
		data_name = string(argv[1]);
		keys_name = string(argv[2]);
	}
	else if(argc == 2){
		num_files = atoi(argv[1]);
	}
	else{
		cout << "Enter data file name: ";
		cin >> data_name;
		cout << "Enter key file name: ";
		cin >> keys_name;
	}

	for(int file=1; file<=num_files; file++){
        int num_keys = 0;
        int num_samples = 0;
		
		//if reading in a bunch of files, update the name
		if(argc==2){
			data_name = "data" + to_string(file) + ".csv";
			keys_name = "keys" + to_string(file) + ".txt";
		}

		//make sure files open properly
		if(!open_file(fin_data, data_name) || !open_file(fin_keys, keys_name)){
			return 1;
		}	

		//ensure proper formatting in both files
		try{
			cout << "\nVerifying files..." << endl;
            num_keys = verify_keys_file(fin_keys);
            cout << "\t" << num_keys << " keys found" << endl;
		    num_samples = verify_data_file(fin_data);
            cout << "\t" << num_samples << " samples found" << endl;
		}catch(invalid_argument& ia){
			cout << ia.what() << endl;
			fin_data.close();
			fin_keys.close();
			continue;
		}

		//close and re-open files to start at beginning
		fin_data.close();
		fin_keys.close();
		if(!open_file(fin_data, data_name) || !open_file(fin_keys, keys_name)){
			continue;
		}
        
        //read two header lines
        getline(fin_data, line);    //Putty header
        //getline(fin_data, line);    //rms, piezo

        vector<char> keys(num_samples, 'x');
        vector< vector<float> > sample(SAMPLE_SIZE, vector<float>(2, 0));
		int id;

		//open output files to append
		ofstream fout_net;
		ofstream fout_rim;
		fout_net.open("net_shot.csv", ios::out | ios::app);
		fout_rim.open("rim_shot.csv", ios::out | ios::app);
        
        get_keys(fin_keys, keys);
        cout << "Keys: ";
        print_keys(keys);

        for(int s=0; s<num_samples; s++){
            id = get_sample(fin_data, sample);
            if(keys[id] == 'n' || keys[id] == 'p'){
                cout << "Writing to net file" << endl;
                write_to_file(fout_net, sample);
            }
            else if(keys[id] == 'r'){
                cout << "Writing to rim file" << endl;
                write_to_file(fout_rim, sample);
            }
            else{
                cout << "Trashing sample" << endl;
            }
        }
	
		cout << "\nDone writing output" << endl;
	
		//close file streams
		fin_data.close();
		fin_keys.close();
		fout_net.close();
		fout_rim.close();

    }

    return 0;
}


//open file with given name
//returns 1 on success, 0 on failure
bool open_file(ifstream& fin, string file_name){
	cout << "Opening " << file_name << endl;
	fin.open(file_name);
	if(!fin.is_open()){
		cout << "\tError opening file" << endl;
		return 0;
	}
	return 1;
}


//keys file should be of the format
//#<num> <identifier>
//where <identifier> is 'n' (net), 'r' (rim), 'p' (pocket), or 'x' (trash)
int verify_keys_file(ifstream& fin_keys){
    string line;
    int i = 0;
    int num;
    size_t space_idx;

    char first = fin_keys.peek();
    if(first == '/'){
        getline(fin_keys, line);
        cout << line << endl;
    }

    while(getline(fin_keys, line)){
        if(line[0] == '\n'){
            cout << "\tFile ended in new line" << endl;
            break;
        }   

        //verify '#' at beginning of line
        if(line[0] != '#' && line[0] != '-'){
            cout << "\tFailed on line " << i << endl;
            throw invalid_argument("\tERROR: no '#' or '-' found at beginning of line");
        }

        //look for space after the number
        space_idx = line.find(' ', 1);
        if(space_idx == string::npos){
            cout << "\tFailed on line " << i << endl;
            throw invalid_argument("\tERROR: no space after number");
        }

        //grab the number, ignoring the '#', before the space
        num = stoi(line.substr(1, space_idx-1));
        
        //check for 'n', 'r', 'p', or 'x' indentifier character
        if((space_idx+1) >= line.length()){
            cout << "\tFailed on line " << i << endl;
            throw invalid_argument("\tERROR: no identifier character given");
        }
        if(line[space_idx+1] != 'r' && line[space_idx+1] != 'n' && line[space_idx+1] != 'p' && line[space_idx+1] != 'x'){
            cout << "\tFailed on line " << i << endl;
            throw invalid_argument("\tERROR: invalid character as identifier");
        }

        i++;
    }

    return i;
}


//data file should be of the format
//<rms_float>,<piezo_int> (repeated for NUM_SAMPLES) followed by #<id> on a new line
int verify_data_file(ifstream& fin_data){
    int i = 0;
    float rms, piezo;
    int idx, num;
    string line;
   
    //read two header lines
    getline(fin_data, line);    //Putty header
    //getline(fin_data, line);    //rms, piezo

    while(!fin_data.eof()){
        //loop through each line in the sample
        for(int j=0; j<SAMPLE_SIZE; j++){ 
            //get line from file
            if(!getline(fin_data, line)){
                //check if there is no more data
                if(j == 0 && fin_data.eof()){
                    return i;
                }
                cout << "\tFailed on sample " << i << " line " << j << endl;
                throw invalid_argument("\tERROR: getline() failed");
            }

            //find comma between numbers
            idx = line.find(',');
            if(idx == string::npos){
                cout << "\tFailed on sample " << i << " line " << j << endl;
                throw invalid_argument("\tERROR: no comma found");
            }

            //grab the numbers before and after the comma and convert to floats
            rms = stof(line.substr(0, idx));
            piezo = stof(line.substr(idx+1));
        }

        //get line with id number
        if(!getline(fin_data, line)){
            cout << "\tFailed on sample " << i << " id line" << endl;
            throw invalid_argument("\tERROR: getline() failed");
        }

        //verify '#' at beginning of line
        if(line[0] != '#'){
            cout << "\tFailed on sample " << i << endl;
            throw invalid_argument("\tERROR: no '#' found at beginning of id line");
        }

        //grab the number, ignoring the '#'
        num = stoi(line.substr(1));

        i++;
    }

    return i;
}


//reads the keys from the key file and stores them into a vector
//each index corresponds to a sample id number
void get_keys(ifstream& fin_keys, vector<char>& keys){
    string line;
    size_t space_idx;
    int num;
    
    char first = fin_keys.peek();
    if(first == '/'){
        getline(fin_keys, line);
    }

    while(getline(fin_keys, line)){
        if(line[0] == '\n'){
            break;
        }   

        //look for space after the number (ignore '#')
        space_idx = line.find(' ', 1);

        //grab the number, ignoring the '#', before the space
        num = stoi(line.substr(1, space_idx-1));

        //store the key for that sample number
        keys[num] = line[space_idx+1];
    }

    return;
}


//print contents of char vector
void print_keys(vector<char> v){
    int n=0, p=0, r=0, x=0;
    for(int i=0; i<v.size(); i++){
        if(v[i] == 'n') n++;
        if(v[i] == 'p') p++;
        if(v[i] == 'r') r++;
        if(v[i] == 'x') x++;
        cout << v[i] << ", ";
    }
    cout << endl;

    cout << "\tNet shots: " << n << "\n\tPocket shots: " << p << "\n\tRim shots: " << r << "\n\tTrash shots: " << x << "\n\tTotal valid shots: " << n+p+r << endl;
}


//read sample from data file and store into 2d vector
//stores both rms and piezo data into vector
//returns id number of sample
int get_sample(ifstream& fin_data, vector< vector<float> >& sample){
    string line;
    int idx;

    //loop through each line in the sample
    for(int i=0; i<SAMPLE_SIZE; i++){
        //get line from file
        getline(fin_data, line);

        //find comma between numbers
        idx = line.find(',');

        //grab the numbers before and after the comma and convert to floats
        sample[i][RMS_IDX] = stof(line.substr(0, idx));
        sample[i][PIEZO_IDX] = stof(line.substr(idx+1));
    }

    //get line with id number
    getline(fin_data, line);

    //return the id number, ignoring the '#'
    return stoi(line.substr(1));
}


//write sample to given file
//writes both rms and piezo data in csv format
void write_to_file(ofstream& fout, vector< vector<float> >& sample){
    for(int i=0; i<SAMPLE_SIZE; i++){
        fout << sample[i][RMS_IDX] << "," << sample[i][PIEZO_IDX] << endl;    
    }
    fout << endl;
}


