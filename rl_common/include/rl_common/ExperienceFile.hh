#ifndef _EXPFILE_HH_
#define _EXPFILE_HH_

#include "Random.h"
#include "core.hh"

#include <vector>
#include <algorithm>

#include <sys/time.h>

class ExperienceFile {
public:
  /** Standard constructor
   */

  std::ofstream vectorFile;
  int expNum;

  ExperienceFile(){
    expNum = 0;
  }

  ~ExperienceFile(){
    if (vectorFile.is_open())
      vectorFile.close();
  }

  void initFile(const char* filename, int nfeats){
    vectorFile.open(filename, std::ios::out | std::ios::binary);

    // first part, save the vector size
    vectorFile.write((char*)&nfeats, sizeof(int));
  }

  void saveExperience(experience e){
    if (!vectorFile.is_open())
      return;

    /*
      if (expNum == 50){
      vectorFile.close();
      return;
      }
    */

    vectorFile.write((char*)&(e.s[0]), e.s.size()*sizeof(float));
    vectorFile.write((char*)&(e.next[0]), e.next.size()*sizeof(float));
    vectorFile.write((char*)&e.act, sizeof(int));
    vectorFile.write((char*)&e.reward, sizeof(float));
    vectorFile.write((char*)&e.terminal, sizeof(bool));

    //std::cout << "Experience " << expNum << std::endl;
    expNum++;
    //printExperience(e);
  }

  void printExperience(experience e){

    std::cout << "State s: ";
    for(unsigned i = 0; i < e.s.size(); i++){
      std::cout << e.s[i] << ", ";
    }
    std::cout << std::endl << " Next: ";
    for(unsigned i = 0; i < e.next.size(); i++){
      std::cout << e.next[i] << ", ";
    }
    std::cout << std::endl;
    std::cout << "action: " << e.act << " reward: " << e.reward << std::endl;

  }



  std::vector<experience> loadExperiences(const char* filename){
    std::ifstream inFile (filename, std::ios::in | std::ios::binary);

    int numFeats;
    inFile.read((char*)&numFeats, sizeof(int));

    std::vector<experience> seeds;

    // while file is not empty
    while(!inFile.eof()){
      experience e;
      e.s.resize(numFeats);
      e.next.resize(numFeats);

      inFile.read((char*)&(e.s[0]), e.s.size()*sizeof(float));
      if (inFile.eof()) break;
      inFile.read((char*)&(e.next[0]), e.next.size()*sizeof(float));
      if (inFile.eof()) break;
      inFile.read((char*)&e.act, sizeof(int));
      inFile.read((char*)&e.reward, sizeof(float));
      inFile.read((char*)&e.terminal, sizeof(bool));

      //std::cout << "Experience " << seeds.size() << std::endl;
      //printExperience(e);

      seeds.push_back(e);
    }

    inFile.close();

    return seeds;
  }

  void closeFile(){
    if (vectorFile.is_open())
      vectorFile.close();
  }

};

#endif
