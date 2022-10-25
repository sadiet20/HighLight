/*
 * Author: Sadie Thomas
 * Date: 4/29/21
 * Description: allows user record how many net/rim shots are detected correctly/incorrectly
 * Inputs: user enters keys to signify correct net shots, wrong net shots (should have been net but detected as rim),
 * 		   correct rim shots, wrong rim shots (should have been rim but detected as net), and questionable shots (cannot 
 * 		   determine if correct)
 * Pre-conditions: None
 * Post-conditions: the totals for each category and percentages of accurracy is appended to "accuracy.txt" along with the current date and time
 */


#include <iostream>
#include <fstream>
#include <ctime>


using namespace std;


int main(){
	
	int correct_net = 0;
	int wrong_net = 0;
	int correct_rim = 0;
	int wrong_rim = 0;
	int questionable = 0;

	cout << "\ncn/a = correct net\nwn/s = wrong net\ncr/d = correct rim\nwr/f = wrong rim\nq = questionable\nendgame = end program\n" << endl;
	string input;
	while(1){
		cout << "Enter: ";
		cin >> input;
		if(input == "cn" || input == "a"){
			correct_net++;
		}
		else if(input == "wn" || input == "s"){
			wrong_net++;
		}
		else if(input == "cr" || input == "d"){
			correct_rim++;
		}
		else if(input == "wr" || input == "f"){
			wrong_rim++;
		}
		else if(input == "q"){
			questionable++;
		}
		else if(input == "endgame"){
			break;
		}
		else{
			cout << "Invalid input!" << endl;
		}
	}

	ofstream fout;
	fout.open("accuracy.txt", ios::out | ios:: app);
	
	time_t now = time(0);

	fout << ctime(&now);
	fout << "Total shots: " << correct_net + wrong_net + correct_rim + wrong_rim << endl;
	fout << "\tCorrect net: " << correct_net << endl;
	fout << "\tWrong net: " << wrong_net << endl;
	fout << "\tCorrect rim: " << correct_rim << endl;
	fout << "\tWrong rim: " << wrong_rim << endl;
	fout << "\tQuestionable: " << questionable << endl << endl;
	fout << "\tNet accuracy: " << float (correct_net)/(correct_net + wrong_net) << endl;
	fout << "\tRim accuracy: " << float (correct_rim)/(correct_rim + wrong_rim) << endl;
	fout << "\tTotal accuracy: " << float (correct_net + correct_rim)/(correct_net + wrong_net + correct_rim + wrong_rim) << endl << endl << endl;

	fout.close();



	return 0;
}
