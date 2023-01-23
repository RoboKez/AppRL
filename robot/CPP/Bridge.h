
void PrintMjVersion(const char *);

int agentGetTotalSteps();
int iterCount(const char * resourcePath, const char * jsonFile);
int* getJSONSteps(const char * resourcePath, const char * jsonFile);
float* getJSONSocs(const char * resourcePath, const char * jsonFile);
float* getJSONDets(const char * resourcePath, const char * jsonFile);

void swiftCancel();
int loadNetwork(int iter, const int agentOnly);
void deleteJson(const char * resourcePath, const char * jsonFile);
void deleteJsonRange(const char * resourcePath, const char * jsonFile, int upperLim);

void deleteAgentandEnv();

const char * listEnvs();
const char * listJsonFiles(const char * resourcePath);

void setEnv(const char * name);
void setSeed(int seed_);
void setH1 (int h1_);
void setH2 (int h2_);
void setLr (float lr_);
void setEpochs (int n_epochs_);
void setMinibatch (int mini_batch_size_);
void setIterSamples (int samples_per_iter_);
void setStepLimit(int step_limit_);
void setAdam(int adam_);
void setNoise(const float noise_);
void setActFilter(const float filter_);


void setPort(const char * port_name_);

const char * getEnv(const char * resourcePath, const char * jsonFile);
const int * getIntData(const char * resourcePath, const char * jsonFile);
float getLr (const char * resourcePath, const char * jsonFile);

const char * listPorts(const char * resourcePath);
int connectSerial(const char * portName);

//void manualReset(const char * resourcePath, const char * portName_);

float* manualStart(const char * resourcePath, const char * portName_);
float* manualStep(float pid_act0, int iter);
void manualTerminate();
int checkUserQuit();
float getReward();

int modelStart(const char * resourcePath, const char * jsonFile, int iter, const char * portName_);
float* modelStep(float* observation_in, float* action_in);

void rlReset();
void manReset();
void rlEpisodeReset(int mode);
int rlStep(int deterministic);
int manStep(float* act_in);

float* getAgentOb();
float* getAct();
float* getEnvOb();
float getReward();
int getDone();
int getQuit();
int getBatchFull();
void rlSave(int iter);
void batchReset();
void rlModel(int iter, int epochs);

void ZeroAdamMomentums();
void UpdatePPO();
void LinearAnneal();

int* getSteps(const char * jsonFile);
float* getStocastic(const char * jsonFile);
float* getDeterministic(const char * jsonFile);
void setFramerate(const int fr_);
void setPathAndFile(const char * resourcePath,
                    const char * jsonFile,
                    const char * jsonSubFile,
                    int subIter);
void manReset();
int getObDim();
float getReturn();
float getSetPoint();
float getSetPoint2();
float getTruePoint();
void StepLimited(const int limit_step);
float* rlPredict(float* ob_in);

void rlMocapReset(const char * env_mode);
void rlMocapEpisodeReset();
int getAgentObDim();
void setWatching(int watch_);

//float* getValueProbe(const char * jsonFile, int iter);
//float* getPolicyProbe(const char * jsonFile, int iter);
float* getNewValueProbe(int save_,int o1, int o2);
float* getNewPolicyProbe(int save_,int o1, int o2);

void ExhaustiveSearch(float minP_, float maxP_, float minI_, float maxI_, int n, int mode_);
float* getExhastiveData();
int getExhaustiveSize();

void setSetPoint(const float sp_);
float* PIProbe(float kP, float kI);
void setDriveMode(int d);
void setKillAngle(float a);
void BasicNN(const char * resourcePath, const char * portName);
void DongleBegin(const char * portName);
void DongleCommand(float byteCommand1, float byteCommand2);

float* getGaitScores();
