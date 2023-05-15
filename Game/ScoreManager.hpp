#pragma once

/* LIBRARY INCLUDES ---------------------------------------------------------*/
#include <iostream>
#include <fstream>
#include <vector>

/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "../config.hpp"

/* STRUCTS ------------------------------------------------------------------*/
typedef struct score_entry_struct{
    std::string name = "anon";
    unsigned long score = 0;
    unsigned long time = 0;
} score_entry_t;

/* CLASSES ------------------------------------------------------------------*/
class ScoreManager
{
    public:
        ScoreManager(const std::string fileName);
        void init();
        std::vector<score_entry_t> getScores(unsigned long start, 
                unsigned long end);
        void addScore(score_entry_t score_entry);
        void close();
        
    protected:
        void readScores();
        void writeScores();
        
        std::string scoreFilePath;
        std::fstream scoreFile;
        std::vector<score_entry_t> scores;
        unsigned long readLineI = 0;
};
