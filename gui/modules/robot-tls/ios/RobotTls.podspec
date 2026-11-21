Pod::Spec.new do |s|
  s.name           = 'RobotTls'
  s.version        = '1.0.0'
  s.summary        = 'Robot REST TLS pinning relay'
  s.description    = 'Probe and 127.0.0.1 relay pinned to the robot SPKI fingerprint'
  s.author         = ''
  s.homepage       = 'https://docs.expo.dev/modules/'
  s.platforms      = {
    :ios => '16.4'
  }
  s.source         = { git: '' }
  s.static_framework = true

  s.dependency 'ExpoModulesCore'

  # Swift/Objective-C compatibility
  s.pod_target_xcconfig = {
    'DEFINES_MODULE' => 'YES',
  }

  s.source_files = "**/*.{h,m,mm,swift,hpp,cpp}"
end
