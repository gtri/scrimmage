import os

root_folder = "./"
folder_queue = ["./" + x for x in os.listdir(root_folder)]

glob_txt = "file(GLOB SRCS *.cpp)"
replace_txt = "${SRCS}"

while len(folder_queue) > 0:
    current_item = folder_queue.pop(0)
    if os.path.isdir(current_item):
        folders = os.listdir(current_item)
        for f in folders:
            folder_queue.append(current_item + "/" + f)
    else:
        if "CMakeLists.txt" in current_item:
            print(current_item)
            with open(current_item, 'r') as file:
                filedata = file.read()
            if glob_txt in filedata:
                filedata = filedata.replace(glob_txt, "")
                print(current_item, current_item.rfind("/"))
                all_cpps = [x + "\n" for x in os.listdir(current_item[0: current_item.rfind("/")]) if ".cpp" in x]
                print("".join(all_cpps))
                filedata = filedata.replace(replace_txt, "".join(all_cpps))
                with open(current_item, 'w') as file:
                    file.write(filedata)

if __name__ == "__main__":
    root_folder = "./"