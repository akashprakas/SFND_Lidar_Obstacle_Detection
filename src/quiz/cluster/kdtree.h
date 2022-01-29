/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
using std::cout;

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
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
//        if(root == nullptr){
////            cout << "Reaching here 1 \n";
//            Node * newNode = new Node(point,id);
//            root = newNode;
//        }
//        // check the depth and revert
//        else{
//             auto x = root->point[0];
////             auto y = point[1];
//             int depth =0;
////            cout << "Reaching here 1 \n";
//             if (root->point[0]>x){ internalInsert(root->right,depth,point,id);}
//             else { internalInsert(root->left,depth,point,id);}
//        }

	}

    void internalInsert(Node * &node,int & depth , std::vector<float> point, int id ){
        if(node == nullptr){
            node = new Node(point,id );
        }
        else {
            auto indexToLook = depth % 2;
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
        if((x> boxCordinates[0][0] && y > boxCordinates[0][1]) && (x < boxCordinates[1][0] && y < boxCordinates[1][1])){
            return true;
        }
        else{
            return false;
        }
    }

	// return a list of point ids in the tree that are within distance of target
    void internalSearch(Node*& node ,std::vector<int>& ids, std::vector<std::vector<float>>& boxCordinates,int depth , std::vector<float>& target ,
                        int distanceTol )
    {

        if (node != nullptr) {
            auto IndexToLook = depth%2;
            depth = depth +1;
            auto point = node->point;
            bool shouldInsert = isInBox(boxCordinates, point);
            if (shouldInsert) {
                float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) +(node->point[1] - target[1])*(node->point[1] - target[1]));
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
        std::vector<std::vector<float>> boxCordinates = {{x - distanceTol,y - distanceTol},{x + distanceTol, y+ distanceTol}};
        // Basic step
        // Loop over the tree and if the point is within the radius then measure the distane else leave. Return the ids that are within the box.
		std::vector<int> ids;
        int depth = 0;
        internalSearch(root,ids,boxCordinates,depth,target,distanceTol);

		return ids;
	}
	

};




