/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
using std::cout;

// Structure to represent node of kd tree
struct KdTree
{
    Node* root;

    KdTree()
            : root(nullptr)
    {}

    ~KdTree()
    {
        delete root;
    }

    void insert(std::vector<float> point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        int depth=0;
        internalInsert(root, depth, point,id);

    }

    void internalInsert(Node * &node,int & depth , std::vector<float> point, int id ){
        if(node == nullptr){
            node = new Node(point,id );
        }
        else {
            auto indexToLook = depth % 3;
            depth = depth + 1;

            auto NodeCheckPoint = node->point[indexToLook];

            if (point[indexToLook] < NodeCheckPoint) {
                internalInsert(node->left, depth, point, id);
            } else {
                internalInsert(node->right, depth, point, id);
            }

        }
    }
// the idea is take [[x1,y1],[x2,y2]] and calculate the
// width = distanceTol
// x1 = xTarget-width, x2 = xTarget + width.
// y1 = yTarget - width, y2 = yTarget + width
    bool isInBox(std::vector<std::vector<float>>& boxCordinates, std::vector<float> point )
    {
        auto x = point[0];
        auto y = point[1];
        auto z = point[2];
        if((x> boxCordinates[0][0] && y > boxCordinates[0][1] && z >boxCordinates[0][2]) && (x < boxCordinates[1][0] && y < boxCordinates[1][1] && z < boxCordinates[1][2])){
            return true;
        }
        else{
            return false;
        }
    }

    // return a list of point ids in the tree that are within distance of target
    void internalSearch(Node*& node ,std::vector<int>& ids, std::vector<std::vector<float>>& boxCordinates,int depth , std::vector<float>& target ,
                        float distanceTol )
    {

        if (node != nullptr) {
            auto IndexToLook = depth%3;
            depth = depth +1;
            auto point = node->point;
            bool shouldInsert = isInBox(boxCordinates, point);
            if (shouldInsert) {
                float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) +(node->point[1] - target[1])*(node->point[1] - target[1]) +(node->point[2] - target[2])*(node->point[2] - target[2]));
                if (distance < distanceTol){
                    ids.push_back(node->id);}
            }

            if(((target[IndexToLook]) - distanceTol) < node->point[IndexToLook]){
                internalSearch(node->left, ids, boxCordinates, depth, target,distanceTol);
            }
            if(((target[IndexToLook]) + distanceTol) > node->point[IndexToLook]){
                internalSearch(node->right, ids, boxCordinates, depth, target,distanceTol);
            }

        }


    }

    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        // Initially create the box coordinates for the isBox function to work
        auto x = target[0];
        auto y = target[1];
        auto z = target[2];
        std::vector<std::vector<float>> boxCordinates = {{x - distanceTol,y - distanceTol, z- distanceTol},{x + distanceTol, y+ distanceTol,z + distanceTol}};
        // Basic step
        // Loop over the tree and if the point is within the radius then measure the distane else leave. Return the ids that are within the box.
        std::vector<int> ids;
        int depth = 0;
        internalSearch(root,ids,boxCordinates,depth,target,distanceTol);

        return ids;
    }

};




