/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <math.h>

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

	void insertHelper(Node **node, uint depth, std::vector<float> point, int id) {
		// tree is empty
		if(*node == NULL) {
			*node = new Node(point, id);
		} else {
			// calculate current dimension
			uint cd = depth % 3; // 3D axist: 0=x axis , 1=y axis, 2=z axis
			if(point[cd] < ((*node)->point[cd])) {
				insertHelper(&((*node)->left), depth+1, point, id);
			} else {
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		Node *node = root;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

	void searchHelper(std::vector<float> &target, Node *node, int depth, float distanceTol, std::vector<int> &ids) {
		int dim = depth % 3;
		if(node != nullptr) {
			if( (node->point[0] <= (target[0]+distanceTol) && node->point[0]>=(target[0]-distanceTol)) && 
				(node->point[1] <= (target[1]+distanceTol) && node->point[1]>=(target[1]-distanceTol)) &&
				(node->point[2] <= (target[2]+distanceTol) && node->point[2]>=(target[2]-distanceTol)) ) {
					float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1])+
											(node->point[2]-target[2])*(node->point[2]-target[2]));
					if(distance <= distanceTol) {
						ids.push_back(node->id);
					}

			}
			// Rec. 1 - if node's cutting axis is intersecting bounding box
			if (node->point[dim] <= (target[dim] + distanceTol) && node->point[dim] >= (target[dim] - distanceTol)) {
				searchHelper(target, node->left, depth+1, distanceTol, ids);
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
			// Rec. 2a - Min bound branch: if node's cutting axis is NOT intersecting bounding box
			else if (node->point[dim] < (target[dim] - distanceTol)) {
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
			// Rec. 2b - Max bound branch: if node's cutting axis is NOT intersecting bounding box
			else {
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}

		}
	}
	
};




