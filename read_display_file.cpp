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