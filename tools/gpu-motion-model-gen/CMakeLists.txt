set (APP_NAME gpu-motion-model-gen)

function(make_includable input_file output_file)
  file(READ ${input_file} content)
  set(delim "for_c++_include")
  set(content "R\"${delim}(${content}})${delim}\"")
  file(WRITE ${output_file} "${content}")
endfunction(make_includable)

make_includable(gpu_motion_model_template.cl generated/gpu_motion_model_template.cl)

add_executable(${APP_NAME} gpu-motion-model-gen.cpp)
