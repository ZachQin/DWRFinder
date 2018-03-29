Pod::Spec.new do |s|
  s.name         = "DWRFinder"
  s.version      = "0.0.1"
  s.summary      = "Dynamic Weather Route Library."
  s.homepage     = "https://github.com/ZachQin/DWRFinder"
  s.license      = "MIT"
  s.author             = { "ZachQin" => "qzkmas@gmail.com" }
  s.ios.deployment_target = "5.0"
  s.osx.deployment_target = "10.7"
  s.source       = { :git => "https://github.com/ZachQin/DWRFinder.git", :tag => s.version }
  s.source_files  = "DWRFinder/DWRCore/*.{h,cc}", "DWRFinder/DWRCore/Utils", "DWRFinder/DWRCore/Utils/*.{h,cc,c}"
  s.public_header_files = "DWRFinder/DWRCore/*.h"
  s.libraries           = 'c++'
end
