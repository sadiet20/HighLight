/*
 * Author: Sadie Thomas
 * Date: 5/23/21
 * Description: parses data and key files, verifies proper formatting, then separates data into two files based on keys
 * Pre-conditions: data file has x sections of SAMPLES (const defined below) lines, each that have 3 numbers (x acceleratrion, y acceleration and z-acceleration),
 * 						at the end of each sample is the word "End" to signify the end of that sample
 * 				   key file has x numbers indicating what the corresponding sample in the data file was (0 = net shot, 1 = rim shot, -1 = garbage)
 * Post-conditions: the formatting (above) is verified in both files and if it correct, it appends each sample's data to either the net_shot.csv or the rim_shot.csv 
 * 					file depending on which key (0 or 1) is specified for that sample. If the -1 key is specified, that sample is not written to either file
 * 					(it is discarded). If one of the files does not have proper formatting the program ends before writing to any files.
 */


#include <iostream>
#include <fstream>
#include <stdexcept>

using namespace std;

#define SAMPLES 450   //may change this 

bool open_file(ifstream&, string);
bool verify_files(ifstream&, ifstream&);
void write_to_file(ifstream&, ofstream&);


int main(int argc, char** argv){
	ifstream fin_data;
	ifstream fin_keys;
	string data_name;
	string keys_name;

	//get file names from command line or user input
	if(argc == 3){
		data_name = string(argv[1]);
		keys_name = string(argv[2]);
	}
	else{
		cout << "Enter data file name: ";
		cin >> data_name;
		cout << "Enter key file name: ";
		cin >> keys_name;
	}

	//make sure files open properly
	if(!open_file(fin_data, data_name) || !open_file(fin_keys, keys_name)){
		return 1;
	}	

	//ensure proper formatting in both files
	try{
		cout << "\nVerifying files..." << endl;
		if(!verify_files(fin_data, fin_keys)){
			return 1;
		}
	}catch(invalid_argument& ia){
		cout << ia.what() << endl;
		fin_data.close();
		fin_keys.close();
		return 1;
	}

	//close and re-open files to start at beginning
	fin_data.close();
	fin_keys.close();
	if(!open_file(fin_data, data_name) || !open_file(fin_keys, keys_name)){
		return 1;
	}

	//open output files to append
	ofstream fout_net;
	ofstream fout_rim;
	fout_net.open("net_shot.csv", ios::out | ios::app);
	fout_rim.open("rim_shot.csv", ios::out | ios::app);

	int key;
	try{	
		cout << endl;
		fin_keys >> key;
		while(!fin_keys.eof()){
			if(key==0){
				cout << "Writing to net file" << endl;
				write_to_file(fin_data, fout_net);
			}
			else if(key==1){
				cout << "Writing to rim file" << endl;
				write_to_file(fin_data, fout_rim);
			}
			else if(key==-1){
				cout << "Trashing output" << endl;
				string trash;
				for(int i=0; i<(SAMPLES*3+1); i++){
					fin_data >> trash;
				}
			}
			else{
				cout << "\tERROR: unidentified key" << endl;
				return 1;	
			}
			fin_keys >> key;
		}
	}catch(invalid_argument& ia){
		cout << ia.what() << endl;
	}

	//read in one more piece of data so eof flag is set
	fin_data >> key;

	cout << "\nDone writing output" << endl;

	//both files should be at the end, otherwise there was an error
	if(fin_data.eof() && fin_keys.eof()){
		cout << "Successful write" << endl;
	}
	else{
		cout << "\tERROR: mismatch in end of file flags" << endl;
		cout << "\tfin_keys.eof(): " << fin_keys.eof() << endl;
		cout << "\tfin_data.eof(): " << fin_data.eof() << endl;
	}	

	//close file streams
	fin_data.close();
	fin_keys.close();
	fout_net.close();
	fout_rim.close();


	return 0;
}


bool open_file(ifstream& fin, string file_name){
	cout << "Opening " << file_name << endl;
	fin.open(file_name);
	if(!fin.is_open()){
		cout << "\tError opening file" << endl;
		return 0;
	}
	return 1;
}


bool verify_files(ifstream& fin_data, ifstream& fin_keys){
	int net = 0;
	int rim = 0;
	int trash = 0;
	string data;
	int key;

	fin_keys >> key;
	while(!fin_keys.eof()){
		//count how many of each shot
		if(key==0){
			net++;
		}
		else if(key==1){
			rim++;
		}
		else if(key==-1){
			trash++;
		}
		else{
			cout << "\tFound key: " << key << endl;
			cout << "\tError on sample number " << net+rim+trash << endl;
			throw invalid_argument("\tERROR: Invalid key detected");
		}

		//make sure all data is present
		for(int i=0; i<(3*SAMPLES); i++){
			fin_data >> data;
			if(fin_data.eof()){
				if(i==0){
					cout << "\tFailed on first line of sample" << endl;
				}
				cout << "\tError on sample number " << net+rim+trash << endl;
				throw invalid_argument("\tERROR: End of data file.");
			}
			if(data == "End"){
				cout << "\tError on sample number " << net+rim+trash << endl;
				throw invalid_argument("\tERROR: End of sample message found too early");
			}
		}

		//check for marker signifying end of sample
		fin_data >> data;
		if(data != "End"){
			cout << "\tError on sample number " << net+rim+trash << endl;
			throw invalid_argument("\tERROR: End of sample message not found");
		}

		fin_keys >> key;
	}

	//read extra data in to set eof flag
	fin_data >> data;

	//make sure both files are at the end
	if(fin_keys.eof() && fin_data.eof()){
		cout << "Files verified. Found " << net+rim+trash << " samples" << endl;
		cout << "\tNet: " << net << endl;
		cout << "\tRim: " << rim << endl;
		cout << "\tTrash: " << trash << endl << endl;
		return 1;
	}
	if(fin_keys.eof()){
		cout << "\tERROR: too much information in data file or not enough in keys file" << endl;
	}
	if(fin_data.eof()){
		cout << "\tERROR: not enough information in data file or too much in keys file" << endl;
	}
	return 0;
}


void write_to_file(ifstream& fin, ofstream& fout){
	string num;
	for(int i=0; i<450; i++){
		for(int j=0; j<3; j++){
			fin >> num;
			if(fin.eof()){
				throw invalid_argument("\tERROR: End of data file. Not enough information in data file or too much in keys file.");
			}
			if(num == "End"){
				throw invalid_argument("\tERROR: End of sample message found too early");
			}
			fout << num;
		}
		fout << endl;
	}
	fout << endl;

	//check for marker signifying end of sample
	fin >> num;
	if(num != "End"){
		throw invalid_argument("\tERROR: End of sample message not found");
	}
}

