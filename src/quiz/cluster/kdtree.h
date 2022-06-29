/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	std::shared_ptr<Node> left;
	std::shared_ptr<Node> right;
	//Node* left;
	//Node* right;

	Node(std::vector<float> arr, int setId)
	//:	point(arr), id(setId), left(NULL), right(NULL)
	:	point(arr), id(setId)
	{}
};

struct KdTree
{
	//Node* root;
	std::shared_ptr<Node> root;
	


	KdTree()
	// : root(NULL)
	{}

	//void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	void insertHelper(std::shared_ptr<Node>* node, uint depth, std::vector<float> point, int id)
	{
		if(*node == nullptr)
		
			*node = std::make_shared<Node>(point, id); 	// If the node is NULL make the current point a new node in the tree.

		else
		{
			//uint cd = depth % 2;
			uint cd = depth % point.size();

			if(point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);

		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertHelper(&root, 0, point, id);

		
	}

	//void insertOptimized(std::vector<float> point, int id)
	//template<typename PointT>
	void insertOptimized( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)	
	{
		
		int size = cloud->points.size();
		for(int id=0; id<size; id++)
		//for(int id=cloud->points.size()-1; id>=0; id--)
		{
			std::shared_ptr<Node>* node = &root;
			uint depth = 0;
			std::vector<float> point{cloud->points[id].x, cloud->points[id].y, cloud->points[id].z};
       		// point.push_back(cloud->points[id].x);
        	// point.push_back(cloud->points[id].y);
        	// point.push_back(cloud->points[id].z);

			uint cd = 0;
			while(*node)
			{
				cd = depth % 3;	
				if(point[cd] <= ((*node)->point[cd]))
				{
					node = &((*node)->left);
				}
				else 
				{
					node = &((*node)->right);
				}
				depth++;
			}
			*node = std::make_shared<Node>(point, id);
		}
	}

	// Search helper takes in target to be evaluated, rood of the tree (starting point), depth, distance tolerance and in
	// indices of the point found within the distance tolerance
	// void searchHelper3D(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids) 
	//void searchHelper3D(std::vector<float> target, std::shared_ptr<Node> node, int depth, float distanceTol, std::vector<int>& ids) 
	void searchHelper3D(std::vector<float> target, std::shared_ptr<Node> node, int depth, float distanceTol, std::vector<int>& ids) 
	{
		if(node != nullptr)
		{
			if( (node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol)) &&
			(node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol)) &&
			(node->point[2] >= (target[2]-distanceTol) && node->point[2] <= (target[2]+distanceTol)) )
			{
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + 
									  (node->point[1]-target[1])*(node->point[1]-target[1]) +
									  (node->point[2]-target[2])*(node->point[2]-target[2]));
				if(distance <= distanceTol)
					ids.push_back(node->id);					
			}

			// Check box boundary
			if( (target[depth % 3]-distanceTol) < node->point[depth % 3] )
				searchHelper3D(target, node->left, depth+1, distanceTol, ids);
			if( (target[depth % 3]+distanceTol) > node->point[depth % 3] )
				searchHelper3D(target, node->right, depth+1, distanceTol, ids);

		}
	}

	// void searchHelper3DOptimized(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids) 
	// void searchHelper3DOptimized(std::vector<float> target, std::shared_ptr<Node> node, int depth, float distanceTol, std::vector<int>& ids) 
	void searchHelper3DOptimized(std::vector<float> target, std::shared_ptr<Node> node, int depth, float distanceTol, std::vector<int>& ids) 
	{
		int k = 0;
		while(node != nullptr)
		{
			k = depth % 3;
			if( (node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol)) &&
			(node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol)) &&
			(node->point[2] >= (target[2]-distanceTol) && node->point[2] <= (target[2]+distanceTol)) )
			{
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + 
									  (node->point[1]-target[1])*(node->point[1]-target[1]) +
									  (node->point[2]-target[2])*(node->point[2]-target[2]));
				if(distance <= distanceTol)
					ids.push_back(node->id);
			}
			if( (target[k]-distanceTol) < node->point[k] )
			{
				node = node->left;
				depth++;
			}

			if( (target[k]+distanceTol) > node->point[k] )
			{
				node = node->right;
				depth++;
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		//searchHelper3DOptimized(target, root, 0 ,distanceTol, ids); 
		searchHelper3D(target, root, 0 ,distanceTol, ids); 

		return ids;
	}
	
	std::vector<int> searchOptimized(std::vector<float> target, float distanceTol)
	{
		// Node* node = root; 
		// int depth = 0;
		// std::vector<int> ids;
		// int k = 0;
		// while(node != NULL)
		// {
		// 	k = depth % 3;
		// 	if( (node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol)) &&
		// 	(node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol)) &&
		// 	(node->point[2] >= (target[2]-distanceTol) && node->point[2] <= (target[2]+distanceTol)) )
		// 	{
		// 		float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + 
		// 							  (node->point[1]-target[1])*(node->point[1]-target[1]) +
		// 							  (node->point[2]-target[2])*(node->point[2]-target[2]));
		// 		if(distance <= distanceTol)
		// 			ids.push_back(node->id);
		// 	}
		// 	if( (target[k]-distanceTol) < node->point[k] )
		// 	{
		// 		node = node->left;
		// 		depth++;
		// 	}

		// 	if( (target[k]+distanceTol) > node->point[k] )
		// 	{
		// 		node = node->right;
		// 		depth++;
		// 	}
		// }

		// return ids;
	}




};




