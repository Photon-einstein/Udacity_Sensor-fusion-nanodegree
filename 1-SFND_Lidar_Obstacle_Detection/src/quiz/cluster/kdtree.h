/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
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
			uint cd = depth % 2;
			if(point[cd] < ((*node)->point[cd])) {
				insertHelper(&((*node)->left), depth+1, point, id);
			} else {
				insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	/* Check if the root of the tree is empty. If it is, create a new node with the given point and ID, and set it as the root.
	If the root is not empty, start traversing the tree to find the appropriate position for the new node.
	Choose whether to compare the x or y coordinate of the point based on the depth of the tree. If the depth is even, compare the x coordinate; if it's odd, compare the y coordinate.
	If the value of the coordinate being compared is less than the value of the current node, move to the left child. If the left child is empty, create a new node with the given point and ID, and set it as the left child.
	If the value of the coordinate being compared is greater than or equal to the value of the current node, move to the right child. If the right child is empty, create a new node with the given point and ID, and set it as the right child.
	Repeat steps 3-5 until you find an empty child node where the new node can be inserted. */
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
		if(node != nullptr) {
			if( (node->point[0] >= (target[0]-distanceTol) && node->point[0]<=(target[0]+distanceTol)) && 
				(node->point[1] >= (target[1]-distanceTol) && node->point[1]<=(target[1]+distanceTol)) ) {
					float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1]));
					if(distance <= distanceTol) {
						ids.push_back(node->id);
					}
			}

			// check cross boundary
			if( (target[depth%2]-distanceTol)<node->point[depth%2] ) {
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			if( (target[depth%2]+distanceTol)>node->point[depth%2] ) {
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}

		}
	}
	
};




