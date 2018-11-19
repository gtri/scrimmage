import pdb
import re


def replace_with_vals():
    test_string = "[ MoveToGoalMS gain='${MS_gain=1.0}' gain2='${MS_gain2=2.543}' gain3='${MS_gain=5.0123}' show_shapes='false' use_initial_heading='true' goal='-1300,0,100']"

    params = ["MS_gain", "MS_gain2"]
    result = test_string
    for param in params:
        reg = r"\${{{}=(.+?)}}".format(param)
        pattern = re.compile(reg)
        # TODO XXX = some param value
        XXX = str(0)
        pdb.set_trace()
        result = pattern.sub(lambda m: m.group().replace(m.group(), XXX), result)
    pdb.set_trace()


def replace_with_default(xml_string):

    #  https://stackoverflow.com/questions/32670413/replace-all-matches-using-re-findall
    #  test = "ab${acde=5}fg${acde=6}"
    #  reg = r"\${{{}cde.+?}}".format('a')
    #  pattern = re.compile(reg)
    #  result = pattern.sub(lambda m: m.group().replace(m.group(), m.group().split("=")[1][:-1]), test)
    #  #  result = pattern.sub(lambda m: m.group().replace(m.group(), (m.group().split("="))[1], test))
    #  match = pattern.findall(test)
    #  #  text = re.search(reg, test)

    #  pdb.set_trace()



    test_string = "[ MoveToGoalMS gain='${MS_gain=1.0}' gain2='${MS_gain2=2.543}' gain3='${MS_gain=5.0123}' show_shapes='false' use_initial_heading='true' goal='-1300,0,100']"

    params = ["MS_gain", "MS_gain2"]
    result = test_string
    result = xml_string
    reg = r"\${.+?=(.+?)}"
    pattern = re.compile(reg)
    result = pattern.sub(lambda m: m.group().replace(m.group(0), m.group(1)), result)




xml_string = ""
with open('../missions/capture-the-flag-test.xml', 'r') as myfile:
    xml_string = myfile.read()


replace_with_default(xml_string)
#  replace_with_vals()
