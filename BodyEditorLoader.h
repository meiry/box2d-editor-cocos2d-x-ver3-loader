#ifndef _BodyEditorLoader_
#define _BodyEditorLoader_

#include "GLES-Render.h"
#include "cJSON.h"
//ported by meir yanovich meiry242@gmail.com
USING_NS_CC; 

// -------------------------------------------------------------------------
// Json Models
// -------------------------------------------------------------------------

struct CircleModel 
{
   b2Vec2 center;
   float radius;
};


struct PolygonModel 
{
    std::list<b2Vec2*> vertices;
	std::vector<b2Vec2*> buffer; // used to avoid allocation in attachFixture()
};


struct RigidBodyModel 
{
    std::string name;
	std::string imagePath;
    b2Vec2 origin;
	std::list<PolygonModel*> polygons;
	std::list<CircleModel*> circles;
};

struct Model 
{
     std::map<std::string,RigidBodyModel>* rigidBodies;
};

class BodyEditorLoader 
{

public:
    typedef std::vector<std::pair<std::string, cJSON> > OrderedMap;
    typedef std::map<std::string,RigidBodyModel>::iterator RigidBodies_iterator; 
    BodyEditorLoader(std::string str);
    BodyEditorLoader();
    ~BodyEditorLoader();
    void init(std::string str);
    Model* readJson(std::string str);
    b2Vec2 getOrigin(std::string name, float scale);
    Model* getInternalModel();
    std::string getImagePath(std::string name);
    void attachFixture(b2Body *body, std::string name, b2FixtureDef* fd, float scale) ;

private:
    // Reusable stuff
	std::list<b2Vec2*> vectorPool;
	b2PolygonShape* polygonShape;
	b2CircleShape circleShape;
	b2Vec2 vec;
    Model* model;

    RigidBodyModel readRigidBody(cJSON *json_bodyElem);
    int GetElementType(int type);
    b2Vec2* newVec();
	void freeVec(b2Vec2* v);
    void printRigidBodyModel(RigidBodyModel rbModel);

};



#endif