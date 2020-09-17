const { description } = require('../../package')

module.exports = {
  /**
   * Ref：https://v1.vuepress.vuejs.org/config/#title
   */
  title: 'Ottobot Docs',
  /**
   * Ref：https://v1.vuepress.vuejs.org/config/#description
   */
  description: description,
  // Base url
  base: '/ottobot/',

  /**
   * Extra tags to be injected to the page HTML `<head>`
   *
   * ref：https://v1.vuepress.vuejs.org/config/#head
   */
  head: [
    ['meta', { name: 'theme-color', content: '#3eaf7c' }],
    ['meta', { name: 'apple-mobile-web-app-capable', content: 'yes' }],
    ['meta', { name: 'apple-mobile-web-app-status-bar-style', content: 'black' }]
  ],

  /**
   * Theme configuration, here is the default theme configuration for VuePress.
   *
   * ref：https://v1.vuepress.vuejs.org/theme/default-theme-config.html
   */
  themeConfig: {
    repo: '',
    editLinks: false,
    docsDir: '',
    editLinkText: '',
    lastUpdated: false,
    nav: [
      {
        text: 'Detail',
        link: '/detail/',
      },
      {
        text: 'Build',
        link: '/build/',
      },
      {
        text: 'Setup',
        link: '/setup/',
      },
      {
        text: 'Blog',
        link: '/blog/2020-04-09_ProjectStart/',
      },
      {
        text: 'About',
        link: '/about/'
      },
      {
        text: 'Github',
        link: 'https://github.com/willhunt/ottobot'
      }
    ],
    sidebar: {
      '/detail/': [
        {
          title: 'Detail',
          collapsable: false,
          children: [
            '',
            'motor_control',
            'battery',
            'imu',
            'kinect'
          ]
        }
      ],
      '/build/': [
        {
          title: 'Build',
          collapsable: false,
          children: [
            '',
            'motor_controller',
            'wild_thumper'
          ]
        }
      ],
      '/setup/': [
        {
          title: 'Setup',
          collapsable: false,
          children: [
            '',
            'arduino',
            'rpi',
            'ros_tools'
          ]
        }
      ],
      '/blog/': [
        {
          title: 'Blog',
          collapsable: false,
          children: [
            '2020-04-09_ProjectStart',
          ]
        }
      ],
      '/about/': [
        {
          title: 'About',
          collapsable: false,
          children: [
            'todo'
          ]
        }
      ]
    }
  },

  /**
   * Apply plugins，ref：https://v1.vuepress.vuejs.org/plugin/
   */
  plugins: [
    '@vuepress/plugin-back-to-top',
    '@vuepress/plugin-medium-zoom',
    ['@vuepress/search', {
      searchMaxSuggestions: 10
    }],
    ['vuepress-plugin-mathjax',
      {
        target: 'svg',
        macros: {
          '*': '\\times',
        },
      },
    ]
  ]
}
