#include "BodyEditorLoader.h"
//ported by meir yanovich meiry242@gmail.com


BodyEditorLoader::BodyEditorLoader(std::string str)
{
   
    polygonShape = new b2PolygonShape();
    model = readJson(str);
     
}

void BodyEditorLoader::init(std::string str)
{
    model = readJson(str);
    
}

BodyEditorLoader::BodyEditorLoader()
{
    
    polygonShape = new b2PolygonShape();    
    
}
BodyEditorLoader::~BodyEditorLoader()
{
}

// -------------------------------------------------------------------------
	// Public API
	// -------------------------------------------------------------------------

	/**
	 * Creates and applies the fixtures defined in the editor. The name
	 * parameter is used to retrieve the right fixture from the loaded file.
	 * <br/><br/>
	 *
	 * The body reference point (the red cross in the tool) is by default
	 * located at the bottom left corner of the image. This reference point
	 * will be put right over the BodyDef position point. Therefore, you should
	 * place this reference point carefully to let you place your body in your
	 * world easily with its BodyDef.position point. Note that to draw an image
	 * at the position of your body, you will need to know this reference point
	 * (see {@link #getOrigin(java.lang.String, float)}.
	 * <br/><br/>
	 *
	 * Also, saved shapes are normalized. As shown in the tool, the width of
	 * the image is considered to be always 1 meter. Thus, you need to provide
	 * a scale factor so the polygons get resized according to your needs (not
	 * every body is 1 meter large in your game, I guess).
	 *
	 * @param body The Box2d body you want to attach the fixture to.
	 * @param name The name of the fixture you want to load.
	 * @param fd The fixture parameters to apply to the created body fixture.
	 * @param scale The desired scale of the body. The default width is 1.
	 */
void BodyEditorLoader::attachFixture(b2Body *body, std::string name, b2FixtureDef* fd, float scale) 
{
		RigidBodyModel rbModel;

		if(model->rigidBodies == NULL || model->rigidBodies->find(name) == model->rigidBodies->end())
		{
			CCLOGWARN("Name %s was not found.",name.c_str());
		}
		else
		{
            RigidBodies_iterator rigidBodies_iterator =  model->rigidBodies->find(name);
            rbModel = rigidBodies_iterator->second;
			
		}
 
		b2Vec2 vec;
		vec.Set(rbModel.origin.x*scale,rbModel.origin.y*scale);
		b2Vec2 origin;
		origin.Set(vec.x,vec.y);
		int poll = 0;
		for (std::list<PolygonModel*>::const_iterator iterator = rbModel.polygons.begin(), end = rbModel.polygons.end(); iterator != end; ++iterator) {
				PolygonModel* polygon = (PolygonModel*)(*iterator);
                b2Vec2 *vertices = new b2Vec2[polygon->vertices.size()];
                //CCLOGWARN("------------- POLYGONE  %d------------",poll); poll++;
				
				int verticeslength = polygon->vertices.size();//polygon->buffer.size();
                int count = 0;
				for (int ii=0, nn=verticeslength; ii<nn; ii++) {
                    ++count;
					b2Vec2* tempVec = newVec();
					b2Vec2* currentVec = (b2Vec2*)polygon->vertices.front();
					polygon->vertices.pop_front();
                    //CCLOGWARN("befor mul(%f) vertices[%d] x:%f y:%f",scale,ii,currentVec->x,currentVec->y);
					tempVec->Set(currentVec->x*scale,currentVec->y*scale); //.mul(scale)
					tempVec->x = tempVec->x;// - origin.x;
					tempVec->y = tempVec->y;// - origin.y;
                    //CCLOGWARN("name:%s after mul(%f) tempVec x:%f y:%f",name.c_str(),scale,ii,tempVec->x,tempVec->y); 
					//vertices[ii].Set(tempVec->x - origin.x ,tempVec->y - origin.y);//vertices[ii].sub(origin);
                    vertices[ii].Set(tempVec->x,tempVec->y);//vertices[ii].sub(origin);
                    //CCLOGWARN("vertices[%d] x:%f y:%f",ii,vertices[ii].x,vertices[ii].y);
					 
				}

				polygonShape->Set(vertices,verticeslength);
                
				fd->shape = polygonShape;
                
				body->CreateFixture(fd);

				for (int ii=0, nn=verticeslength; ii<nn; ii++) {
					b2Vec2 *tmp = new b2Vec2(vertices[ii].x,vertices[ii].y);						
					freeVec(tmp);
				}

				delete[] vertices;


		}

		for (std::list<CircleModel*>::const_iterator iterator = rbModel.circles.begin(), end = rbModel.circles.end(); iterator != end; ++iterator) {
				CircleModel* circle = (CircleModel*)(*iterator);
				b2Vec2* center = newVec();
				center->Set(circle->center.x*scale,circle->center.y*scale);
				float radius = circle->radius * scale;
				circleShape.m_p.Set(center->x,center->y);
				circleShape.m_radius= radius;
				fd->shape = &circleShape;
				body->CreateFixture(fd);
				freeVec(center);
		}
}

// -------------------------------------------------------------------------
	// Json reading process
	// -------------------------------------------------------------------------

Model* BodyEditorLoader::readJson(std::string str) {
	//Model m = new Model();
    Model* m = new Model();
    m->rigidBodies =   new  std::map<std::string,RigidBodyModel>();
    //m->rigidBodies = new std::map<std::string,RigidBodyModel>();
	cJSON *json_root;
    json_root=cJSON_Parse(str.c_str());
    if (!json_root) {CCLOGWARN("Error before: [%s]\n",cJSON_GetErrorPtr());}
	else
	{
	
      
        cJSON *json_bodiesElems = cJSON_GetObjectItem(json_root,"rigidBodies"); 
        
        int i; 
        for (i=0;i<cJSON_GetArraySize(json_bodiesElems);i++)
	    {
		    cJSON *json_bodyElem=cJSON_GetArrayItem(json_bodiesElems,i);
            RigidBodyModel rbModel = readRigidBody(json_bodyElem);
            printRigidBodyModel(rbModel);
            m->rigidBodies->insert(std::pair<std::string,RigidBodyModel>(rbModel.name,rbModel));
		
	    } 
        
	}
     
 	return m;
}

RigidBodyModel BodyEditorLoader::readRigidBody(cJSON *json_bodyElem) {
		
        RigidBodyModel rbModel;
        std::string str_name(cJSON_GetObjectItem(json_bodyElem,"name")->valuestring); 
        std::string str_imagePath(cJSON_GetObjectItem(json_bodyElem,"imagePath")->valuestring); 
		rbModel.name = str_name;
		rbModel.imagePath = str_imagePath;

		cJSON* originElem = cJSON_GetObjectItem(json_bodyElem,"origin");
        rbModel.origin.x = (float)cJSON_GetObjectItem(originElem,"x")->valuedouble;
		rbModel.origin.y = (float)cJSON_GetObjectItem(originElem,"y")->valuedouble;

		// polygons

		cJSON* polygonsElem = cJSON_GetObjectItem(json_bodyElem,"polygons");

		for (int i=0; i<cJSON_GetArraySize(polygonsElem); i++) {
			PolygonModel *polygon = new PolygonModel() ;
            rbModel.polygons.push_back(polygon);

			cJSON *json_verticesElem = cJSON_GetArrayItem(polygonsElem,i);
            
			for (int ii=0; ii<cJSON_GetArraySize(json_verticesElem); ii++) {
				
                cJSON *json_vertexElem = cJSON_GetArrayItem(json_verticesElem,ii);
                float x = (float)cJSON_GetObjectItem(json_vertexElem,"x")->valuedouble;
		        float y = (float)cJSON_GetObjectItem(json_vertexElem,"y")->valuedouble; 
				 
                polygon->vertices.push_back(new b2Vec2(x, y));
			}

            polygon->buffer.reserve(polygon->vertices.size());  // needs to be simple int
		}

		// circles
        
        cJSON* circlesElem = cJSON_GetObjectItem(json_bodyElem,"circles");
        if(circlesElem!=NULL && cJSON_GetArraySize(circlesElem)>0)
        {            
		    for (int i=0; i<cJSON_GetArraySize(circlesElem); i++) {
			     CircleModel* circle = new CircleModel() ;
                 cJSON *jsoncircleElem = cJSON_GetArrayItem(circlesElem,i);                 
			     circle->center.x = (float)cJSON_GetObjectItem(jsoncircleElem,"cx")->valuedouble;;
			     circle->center.y = (float)cJSON_GetObjectItem(jsoncircleElem,"cy")->valuedouble;
			     circle->radius = (float)cJSON_GetObjectItem(jsoncircleElem,"r")->valuedouble;
                 rbModel.circles.push_front(circle); 
		    }
        }

		return rbModel;
	}

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------
/**
	 * Gets the image path attached to the given name.
	 */
std::string BodyEditorLoader::getImagePath(std::string name) 
{
	RigidBodyModel rbModel;
    std::string imagemap="";
    
    if(model->rigidBodies == NULL || model->rigidBodies->find(name) == model->rigidBodies->end())
    {
        CCLOGWARN("Name %s was not found.",name.c_str());
    }
    else
    {
       RigidBodies_iterator rigidBodies_iterator =  model->rigidBodies->find(name);
       rbModel = rigidBodies_iterator->second;
       imagemap = rbModel.imagePath;
        
       
    }

	return rbModel.imagePath;
}

/**
	* Gets the origin point attached to the given name. Since the point is
	* normalized in [0,1] coordinates, it needs to be scaled to your body
	* size. Warning: this method returns the same Vector2 object each time, so
	* copy it if you need it for later use.
	*/
 


b2Vec2 BodyEditorLoader::getOrigin(std::string name, float scale)
{
	RigidBodyModel rbModel;
	RigidBodies_iterator rigidBodies_iterator =  model->rigidBodies->find(name);
    rbModel = rigidBodies_iterator->second; 
    if(model->rigidBodies == NULL || model->rigidBodies->find(name) == model->rigidBodies->end())
    {
        CCLOGWARN("Name %s was not found.",name.c_str());
    }

    b2Vec2 vec;
    vec.Set(rbModel.origin.x*scale,rbModel.origin.y*scale);
    //vec.Set(rbModel.origin.x,rbModel.origin.y);
    return vec;
	
}


/**
	* <b>For advanced users only.</b> Lets you access the internal model of
	* this loader and modify it. Be aware that any modification is permanent
	* and that you should really know what you are doing.
	*/
Model* BodyEditorLoader::getInternalModel()
{
	return model;
}
b2Vec2* BodyEditorLoader::newVec() 
{
    int vectorPoolSize = vectorPool.size();
    if(vectorPoolSize==0)
    {
       return new b2Vec2();
    }
    else
    {
        
        b2Vec2* vec = (b2Vec2*)vectorPool.front();
        vectorPool.erase(vectorPool.begin());
        return vec;
    }
}

void BodyEditorLoader::freeVec(b2Vec2* v) {
    vectorPool.push_front(v);
}

int BodyEditorLoader::GetElementType(int type)
{
    int typeNum = -1;
    switch ((type)&255)
	    {
		case cJSON_NULL:
            {
           typeNum = cJSON_NULL;	
            break;
            }
		case cJSON_False:	
            {
            typeNum = cJSON_False;		
            break;
            }
		case cJSON_True:	
           {
           typeNum = cJSON_True;	
           break;
           }
		case cJSON_Number:	
           {
           typeNum = cJSON_Number;	
           break;
           }
		case cJSON_String:	
           {
           typeNum = cJSON_String;	
           break;
           }
		case cJSON_Array:	
            {
            typeNum = cJSON_Array;	
            break; 
            }
		case cJSON_Object:	
            {
            typeNum = cJSON_Object;	
            break; 
            }
	    }
    return typeNum;
}


void BodyEditorLoader::printRigidBodyModel(RigidBodyModel rbModel)
{
    /*
    struct RigidBodyModel 
    {
    std::string name;
	std::string imagePath;
    b2Vec2 origin;
	std::list<PolygonModel*> polygons;
	std::list<CircleModel*> circles;
    };
    */

   // CCLOGWARN("name:%s imagePath:%s",rbModel.name.c_str(),rbModel.imagePath.c_str());
    int pol =0;
    for (std::list<PolygonModel*>::const_iterator iterator = rbModel.polygons.begin(), end = rbModel.polygons.end(); iterator != end; ++iterator)
    {

				 
               // CCLOGWARN("------------- POLYGONE %d------------",pol); pol++;
                PolygonModel* polygonModel = (PolygonModel*)(*iterator);
                for (std::list<b2Vec2*>::const_iterator iterator = polygonModel->vertices.begin(), end = polygonModel->vertices.end(); iterator != end; ++iterator)
                {
                    b2Vec2* b3vec = (b2Vec2*)(*iterator);
                    //CCLOGWARN("x:%f x:%f",b3vec->x,b3vec->y);
                }
               
    }

}