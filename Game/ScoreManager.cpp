/* CUSTOM INCLUDES ----------------------------------------------------------*/
#include "ScoreManager.hpp"

/* METHODS ------------------------------------------------------------------*/
ScoreManager::ScoreManager(const std::string fileName)
{
    this->scoreFilePath = fileName;
}

void ScoreManager::init()
{
    this->scoreFile.open(this->scoreFilePath, std::fstream::in);

    if(!this->scoreFile.is_open()){
        return;
    }

    this->readScores();
    this->scoreFile.close();
}

void ScoreManager::close()
{
    this->writeScores();
}

void ScoreManager::readScores()
{
    if(!this->scoreFile.is_open()){
        std::cerr << "Score file is not open for reading!" << std::endl;
    }
    
    std::string line = "";
    for(unsigned long i = 0; i < MAX_SCORE_ENTRIES &&
            std::getline(this->scoreFile, line); i++){
        std::vector<std::string> fields;

        score_entry_t score_entry;
        unsigned long delimPos = 0;

        /* Get the fields */
        while((delimPos = line.find(SCORE_FILE_DELIM)) != std::string::npos){
            std::string field = line.substr(0, delimPos);
            fields.push_back(field);
            line.erase(0, delimPos + SCORE_FILE_DELIM.length());
        }
        fields.push_back(line);

        if(fields.size() != 3){
            i--;
            continue;
        }

        try{
            score_entry.name = fields.at(0);
            score_entry.score = std::stoul(fields.at(1));
            score_entry.time = std::stoul(fields.at(2));
        }catch(std::exception &e){
            i--;
            continue;
        }

        this->scores.push_back(score_entry);
    }
}

void ScoreManager::writeScores()
{
    unsigned long size = (this->scores.size() < MAX_SCORE_ENTRIES) ? 
        this->scores.size() : MAX_SCORE_ENTRIES;

    if(!size){
        return;
    }
    
    this->scoreFile.open(this->scoreFilePath, std::fstream::out);
    
    if(!this->scoreFile.is_open()){
        std::cerr << "Cannot open/create score file for writing!" << std::endl;
        return;
    }

    for(unsigned long i = 0; i < size; i++){
        this->scoreFile << this->scores[i].name << ";" <<
            this->scores[i].score << ";" << this->scores[i].time << std::endl;
    }
    
    this->scoreFile.close();
}

std::vector<score_entry_t> ScoreManager::getScores(unsigned long start,
        unsigned long end)
{
    if(this->scores.size() < start+1 || !end){
        return {};
    }if(this->scores.size() < end){
        end = this->scores.size();
    }

    std::vector<score_entry_t> reqScores;
    for(; start < end; start++){
        reqScores.push_back(this->scores[start]);
    }

    return reqScores;
}

void ScoreManager::addScore(score_entry_t score_entry)
{
    if(!score_entry.time){
        return;
    }
    
    unsigned long size = this->scores.size();
    if(!size || (this->scores.back().score > score_entry.score && 
                size < MAX_SCORE_ENTRIES)){
        this->scores.push_back(score_entry);
        return;
    }
    
    for(std::vector<score_entry_t>::iterator it = this->scores.begin();
            it != this->scores.end(); it++){
        if(score_entry.score > it->score){
            this->scores.insert(it, score_entry);
            break;
        }
    }

    while(this->scores.size() > MAX_SCORE_ENTRIES){
        this->scores.pop_back();
    }
}

/*unsigned long ScoreManager::findScorePos(score_entry_t score_entry)
{
    unsigned long page_i = 0;
    std::vector<score_entry_t> scores = this->readScores(0);

    for(; scores.size() == SCORE_ENTRIES_NUM && 
            score_entry.score < scores.back().score; page_i++){
        scores = this->readScores(page_i*SCORE_ENTRIES_NUM);
    }
    page_i = page_i ? page_i-1 : 0;

    unsigned long scoresSize = scores.size();
    unsigned long i = scores.size() ? scores.size() - 1 : 0;
    unsigned long j = i; 
    while(i > 0 && score_entry.score > scores[i].score){
        j = i;
        i /= 2;
    }

    unsigned long oldI = i;
    for(; i <= j && scores[i].score > score_entry.score; i++);

    return (page_i * SCORE_ENTRIES_NUM + i);
}*/
