// qsort.cpp: 定义控制台应用程序的入口点。
//
#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct
{
	char name[10];
	int chinese;
	int math;
	int eng;
}Student;

Student student[5];

void read_data()
{
	FILE* file;
	fopen_s(&file, "1.txt", "r");
	for (int i = 0; i < 5; i++)
	{
		fscanf_s(file, "%s", student[i].name, _countof(student[i].name));
		fscanf_s(file, "%d", &student[i].chinese);
		fscanf_s(file, "%d", &student[i].math);
		fscanf_s(file, "%d", &student[i].eng);
	}
	fclose(file);
}

void display_data()
{
	for (int i = 0; i < 5; i++)
	{
		printf("%s\t", student[i].name);
		printf("%d\t", student[i].chinese);
		printf("%d\t", student[i].math);
		printf("%d\n", student[i].eng);
	}
	printf("\n");
}

int compare(const void* a, const void* b)
{
	Student* pa = (Student*)a;
	Student* pb = (Student*)b;
	int num1 = pa->math;
	int num2 = pb->math;
	return num1 - num2;
}

int compare2(const void* key, const void* e)
{
	int* p1 = (int*)key;
	int num1 = *p1;
	Student* p2 = (Student*)e;
	int num2 = p2->math;
	return num1 - num2;
}

int main()
{
	read_data();
	display_data();
	qsort(student, 5, sizeof(Student), compare);
	display_data();
	int key = 99;
	Student* s = (Student*)bsearch(&key, student, 5, sizeof(Student), compare2);
	printf("%s\n", s->name);
	return 0;
}

