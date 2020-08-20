/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node *&node, int depth, std::vector<float> point, int id)
	{
		if(node == NULL){
			node = new Node(point, id);
		}
		else{
			int ind = depth % point.size();
			if(node->point[ind] > point[ind]){
				insertHelper(node->left,depth+1,point, id);
			}
			else{
				insertHelper(node->right,depth+1,point, id);
			}
		}
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		if(root == NULL){
			root= new Node(point,id);
		} 
		else{
			insertHelper(root,0,point,id);
		}

	}

	void searchHelper(Node *&node, int depth,std::vector<float> target, float distanceTol, std::vector<int> &ids)
	{
		if(node == NULL){
			return;
		}
		else{
			if(target.size() == 2){
				if(abs(node->point[0] - target[0]) <= distanceTol && abs(node->point[1] - target[1]) <= distanceTol){
					float dist = (node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]);
					if(dist <= distanceTol*distanceTol){
						ids.push_back(node->id);
					}
					// searchHelper(node->left,depth+1,target, distanceTol,ids);
					// searchHelper(node->right,depth+1,target, distanceTol,ids);
				}
			}
			else if(target.size() == 3){
				if(abs(node->point[0] - target[0]) <= distanceTol && abs(node->point[1] - target[1]) <= distanceTol && abs(node->point[2] - target[2]) <= distanceTol){
					float dist = (node->point[0] - target[0])*(node->point[0] - target[0]) 
								+ (node->point[1] - target[1])*(node->point[1] - target[1]) 
								+ (node->point[2] - target[2])*(node->point[2] - target[2]);
					if(dist <= distanceTol*distanceTol){
						ids.push_back(node->id);
					}
					// searchHelper(node->left,depth+1,target, distanceTol,ids);
					// searchHelper(node->right,depth+1,target, distanceTol,ids);
				}
			}			

			int ind = depth % target.size();
			if(node->point[ind] > (target[ind] - distanceTol)){
				searchHelper(node->left,depth+1,target, distanceTol,ids);
			}
			if(node->point[ind] < (target[ind] + distanceTol)){
				searchHelper(node->right,depth+1,target, distanceTol,ids);
			}
		
			
		}
	}
	

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root,0, target, distanceTol, ids);
		return ids;
	}
	

};




