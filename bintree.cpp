//    二叉树的实现（C语言）
//    链表，递归实现
//    编译环境：visual studio 2017
//    操作系统：win8.1

#include "stdafx.h"
#include<stdio.h>
#include<stdlib.h>

typedef struct node
{
	int data;
	struct node *left;
	struct node *right;
}Node;

typedef struct 
{	
	Node *root;
}Tree;

void preorder(Node *node)
{
	if (node != NULL)
	{
		printf("%d\n", node->data);
		preorder(node->left);
		preorder(node->right);
	}
}

void inorder(Node *node)
{
	if (node != NULL)
	{
		inorder(node->left);
		printf("%d\n", node->data);
		inorder(node->right);
	}
}

void postorder(Node *node)
{
	if (node != NULL)
	{
		postorder(node->left);
		postorder(node->right);
		printf("%d\n", node->data);
	}
}

void insert(Tree* tree, int val)
{
	Node* node = (Node*)malloc(sizeof(Node));
	node->data = val;
	node->left = NULL;
	node->right = NULL;

	if (tree->root == NULL)
	{
		tree->root = node;
	}
	else
	{
		Node* tmp = tree->root;
		while (tmp != NULL)
		{
			if (val < tmp->data)
			{
				if (tmp->left == NULL)
				{
					tmp->left = node;
					return;
				}
				else
				{
					tmp = tmp->left;
				}
			}
			else
			{
				if (tmp->right == NULL)
				{
					tmp->right = node;
					return;
				}
				else
				{
					tmp = tmp->right;
				}
			}
		}
	}

}

int get_height(Node* node)
{
	if (node == NULL)
		return 0;
	else
	{
		int left_height = get_height(node->left);
		int right_height = get_height(node->right);
		int max = left_height;
		if (right_height > max)
			max = right_height;
		return max + 1;
	}
}

int get_maximum(Node* node)
{
	if (node == NULL)
		return -1;
	else
	{
		int m1 = get_maximum(node->left);
		int m2 = get_maximum(node->right);
		int m3 = node->data;
		int max = m1;
		if (m2 > max)	max = m2;
		if (m3 > max)	max = m3;
		return max;
	}
}
int main()
{
	/*Node n1, n2, n3, n4;

	n1.data = 5;
	n2.data = 6;
	n3.data = 7;
	n4.data = 8;

	n1.left = &n2;
	n1.right = &n3;
	n2.left = &n4;
	n2.right = NULL;
	n3.left = NULL;
	n3.right = NULL;
	n4.left = NULL;
	n4.right = NULL;
	
	inorder(&n1);*/

	int arr[7] = { 6, 3, 8, 2, 5, 1, 7 };
	Tree tree;
	tree.root = NULL;

	for (int i = 0; i < 7; i++)
	{
		insert(&tree, arr[i]);
	}
	inorder(tree.root);
	int h = get_height(tree.root);
	printf("h = %d\n", h);
	int n = get_maximum(tree.root);
	printf("m = %d\n", n);

};
