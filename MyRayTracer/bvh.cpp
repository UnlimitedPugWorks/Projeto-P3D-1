#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_;
	this->n_objs = n_objs_;
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_;
	//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object*>& objs) {


	BVHNode* root = new BVHNode();

	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB world_bbox = AABB(min, max);

	for (Object* obj : objs) {
		AABB bbox = obj->GetBoundingBox();
		world_bbox.extend(bbox);
		objects.push_back(obj);
	}
	world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
	world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
	root->setAABB(world_bbox);
	nodes.push_back(root);
	build_recursive(0, objects.size(), root); // -> root node takes all the 
	//printf("num_of_nodes:%d\n", nodes.size());
	int num_leafs = 0;
	for (int i = 0; i < nodes.size(); i++) {
		if (nodes[i]->isLeaf()) {
			num_leafs += 1;
		}
	}
	//printf("num_leafs:%d\n", num_leafs);
}

void BVH::build_recursive(int left_index, int right_index, BVHNode* node) {

	int num_objs = (right_index - left_index);

	//printf("num_objes:%d\n", num_objs);

	if (num_objs <= Threshold) {
		node->makeLeaf(left_index, num_objs);
	}
	else {

		AABB aabb = node->getAABB();

		int dim = -1;

		Vector diff = aabb.max - aabb.min;


		if (diff.x >= diff.y && diff.x >= diff.z) {
			dim = 0;
		}
		else if (diff.y >= diff.x && diff.y >= diff.z) {
			dim = 1;
		}
		else {
			dim = 2;
		}

		Comparator cmp;
		cmp.dimension = dim;

		sort(objects.begin() + left_index, objects.begin() + right_index, cmp);


		float mid = (aabb.max.getAxisValue(dim) + aabb.min.getAxisValue(dim)) * 0.5;

		int split_index;

		//Make sure that neither left nor right is completely empty
		if (objects[left_index]->getCentroid().getAxisValue(dim) > mid ||
			objects[right_index - 1]->getCentroid().getAxisValue(dim) <= mid) {
			split_index = left_index + num_objs / 2;
		}


		//Split intersectables objects into left and right by finding a split_index

		else {
			for (split_index = left_index; split_index < right_index; split_index++) {
				if (objects[split_index]->getCentroid().getAxisValue(dim) > mid) {
					break;
				}
			}
		}

		//Create two new nodes, leftNode and rightNode and assign bounding boxes
		Vector min_right, min_left;
		min_right = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
		min_left = min_right;
		Vector max_right, max_left;
		max_right = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		max_left = max_right;

		AABB leftBox = AABB(min_left, max_left);
		AABB rightBox = AABB(min_right, max_right);

		for (int left = left_index; left < split_index; left++) {
			leftBox.extend(objects[left]->GetBoundingBox());
		}

		for (int right = split_index; right < right_index; right++) {
			rightBox.extend(objects[right]->GetBoundingBox());
		}


		// Create two new nodes, leftNode and rightNode and assign bounding boxes
		BVHNode* leftNode = new BVHNode();
		BVHNode* rightNode = new BVHNode();


		leftNode->setAABB(leftBox);

		rightNode->setAABB(rightBox);

		//Initiate current node as an interior node with leftNode and rightNode as children: 
		node->makeNode(nodes.size());

		//Push back leftNode and rightNode into nodes vector 
		nodes.push_back(leftNode);
		nodes.push_back(rightNode);

		build_recursive(left_index, split_index, leftNode);
		build_recursive(split_index, right_index, rightNode);

	}

}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
	float tmp;
	float tmp2;
	float tmin = FLT_MAX;  //contains the closest primitive intersection
	bool hit = false;

	Ray LocalRay = ray;
	BVHNode* currentNode = nodes[0];
	Object* ClosestObj = NULL;

	AABB bbox = currentNode->getAABB();

	if (!bbox.intercepts(LocalRay, tmp)) {
		return(false);
	}

	while (true) {
		if (!currentNode->isLeaf()) {
			int index = currentNode->getIndex();
			BVHNode* leftChild = nodes[index];
			BVHNode* rightChild = nodes[index + 1];
			AABB bboxLeft = leftChild->getAABB();
			AABB bboxRight = rightChild->getAABB();

			bool leftHit = bboxLeft.intercepts(LocalRay, tmp);
			bool rightHit = bboxRight.intercepts(LocalRay, tmp2);

			//Test if inside
			if (bboxLeft.isInside(ray.origin)) tmp = 0;
			if (bboxRight.isInside(ray.origin)) tmp2 = 0;

			if (leftHit && rightHit) {
				if (tmp < tmp2) {
					currentNode = leftChild;
					hit_stack.push(StackItem(rightChild, tmp2));
					continue;
				}
				else {
					currentNode = rightChild;
					hit_stack.push(StackItem(leftChild, tmp));
					continue;
				}
			}
			else if (leftHit) {
				currentNode = leftChild;
				continue;
			}
			else if (rightHit) {
				currentNode = rightChild;
				continue;
			}
		}
		else {
			int index = currentNode->getIndex();
			int numObjs = currentNode->getNObjs();
			float curr_tmp;
			for (int i = index; i < (index + numObjs); i++) {
				if (objects[i]->intercepts(LocalRay, curr_tmp) && curr_tmp < tmin) {
					tmin = curr_tmp;
					ClosestObj = objects[i];
				}
			}
		}

		bool changed = false;


		while (!hit_stack.empty()) {
			StackItem item = hit_stack.top();
			hit_stack.pop();
			if (item.t < tmin) {
				currentNode = item.ptr;
				changed = true;
				break;
			}
		}

		if (changed) { continue; }

		if (hit_stack.empty()) {
			if (ClosestObj != NULL) {
				*hit_obj = ClosestObj;
				hit_point = ray.origin + ray.direction * tmin;
				return true;
			}
			else {
				return false;
			}
		}
	}
}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
	float tmp;
	float tmp2;

	double length = ray.direction.length(); //distance between light and intersection point
	ray.direction.normalize();

	//Local Ray = Ray
	Ray LocalRay = ray;
	//CurrentNode = nodes[0];
	BVHNode* currentNode = nodes[0];
	
	//Check LocalRay intersection with Root(world box)
	AABB bbox = currentNode->getAABB();
	//No hit = > return false
	if (!bbox.intercepts(LocalRay, tmp)) {
		return(false);
	}
	//For Infinity
	while (true) {
		// If (NOT CurrentNode.isLeaf())
		if (!currentNode->isLeaf()) {
			int index = currentNode->getIndex();
			BVHNode* leftChild = nodes[index];
			BVHNode* rightChild = nodes[index + 1];

			AABB bboxLeft = leftChild->getAABB();
			AABB bboxRight = rightChild->getAABB();

			//Intersection test with both child nodes
			bool leftHit = bboxLeft.intercepts(LocalRay, tmp);
			bool rightHit = bboxRight.intercepts(LocalRay, tmp2);

			if (leftHit && rightHit) {
				//Both nodes hit => Put right one on the stack. CurrentNode = left node
				hit_stack.push(StackItem(rightChild, tmp2));
				currentNode = leftChild;
				//Goto LOOP;
				continue;
			}
			//Only one node hit = > CurrentNode = hit node
			else if (leftHit) {
				currentNode = leftChild;
				//Goto LOOP;
				continue;
			}
			//Only one node hit = > CurrentNode = hit node
			else if (rightHit) {
				currentNode = rightChild;
				//Goto LOOP;
				continue;
			}
			else {
				//No Hit: Do nothing (let the stack-popping code below be reached)
			}
		}
		//Else Is leaf
		else {
			int index = currentNode->getIndex();
			float curr_tmp;
			int numObjs = currentNode->getNObjs();
			//For each primitive in leaf perform intersection testing
			for (int i = index; i < (index + numObjs); i++) {
				if (objects[i]->intercepts(LocalRay, curr_tmp) && curr_tmp < length) {
					//Intersected => return true;
					return true;
				}
			}
		}
		//End If
		bool changed = false;

		if (!hit_stack.empty()) {
			//Pop stack, CurrentNode = pop’d node
			StackItem item = hit_stack.top();
			hit_stack.pop();
			currentNode = item.ptr;
			changed = true;
		}

		if (changed) { continue; }

		//Stack is empty = > return false
		if (hit_stack.empty()) { return false; }

		//EndFor
	}
}